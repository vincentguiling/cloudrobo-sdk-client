#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""飞书文档同步工具：本地 Markdown → 飞书 docx，就地更新，绝不重复建副本。

用法（在仓库任意 worktree 根目录执行，或传绝对路径）：
  python tools/feishu_doc_sync.py check                      # 体检：登记表/重复副本/标题一致性
  python tools/feishu_doc_sync.py sync --path <md 路径>       # 就地更新（默认拒绝新建，防重复副本）
  python tools/feishu_doc_sync.py sync --path <md> --allow-create  # 确认是全新文档才新建并登记
  python tools/feishu_doc_sync.py sync --path <md> --dry-run  # 只打印计划，不落盘
  python tools/feishu_doc_sync.py sync --path <md> --force    # 允许覆盖时丢弃文档内嵌媒体（默认拒绝）

前置：lark-cli 已按 bot 身份登录。
登记表：docs/feishu-docs.json（token 的唯一真源；标题匹配是兜底，不是替代）。
"""

import argparse
import datetime
import hashlib
import json
import pathlib
import re
import shutil
import subprocess
import sys

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
REGISTRY_PATH = REPO_ROOT / "docs" / "feishu-docs.json"
TMP_MD = REPO_ROOT / ".feishu-sync-tmp.md"
TMP_METAS = REPO_ROOT / ".feishu-sync-metas.json"
TMP_PERM = REPO_ROOT / ".feishu-sync-perm.json"
EXIT = {"code": 0}


def out(msg: str) -> None:
    print(msg, flush=True)


def fail(msg: str) -> None:
    out(f"[FAIL] {msg}")
    EXIT["code"] = 1


class LarkError(RuntimeError):
    pass


def lark(args, allow_failure=False):
    """运行 lark-cli 并解析 JSON；allow_failure=True 时失败返回原 JSON 或 None。"""
    exe = shutil.which("lark-cli")
    if exe is None:
        raise LarkError("PATH 上找不到 lark-cli")
    proc = subprocess.run(
        [exe, *args],
        cwd=str(REPO_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
    )
    text = (proc.stdout or "").strip()
    if not text:
        if proc.returncode == 0:
            return {"ok": True, "data": {}}
        if allow_failure:
            return None
        err = (proc.stderr or "").strip()
        raise LarkError(f"lark-cli 无输出（rc={proc.returncode}）：lark-cli {' '.join(args)} => {err[-300:]}")
    try:
        data = json.loads(text)
    except json.JSONDecodeError:
        if allow_failure:
            return None
        raise LarkError(f"lark-cli 输出不是 JSON：lark-cli {' '.join(args)}")
    if not data.get("ok"):
        if allow_failure:
            return data
        err = (data.get("error") or {}).get("message") or text.replace("\n", " ")[:300]
        raise LarkError(f"lark-cli 失败：lark-cli {' '.join(args)} => {err}")
    return data


def load_registry():
    if not REGISTRY_PATH.exists():
        raise LarkError(f"登记表不存在：{REGISTRY_PATH}")
    # utf-8-sig：容忍外部编辑器写入的 BOM
    return json.loads(REGISTRY_PATH.read_text(encoding="utf-8-sig"))


def save_registry(reg):
    REGISTRY_PATH.write_text(json.dumps(reg, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def doc_title(md_text):
    m = re.search(r"^#\s+(.+?)\s*$", md_text, flags=re.MULTILINE)
    return m.group(1) if m else None


def local_doc(raw_path):
    if not raw_path:
        raise LarkError("sync 需要本地文档路径（--path）")
    full = pathlib.Path(raw_path)
    if not full.is_absolute():
        full = REPO_ROOT / full
    full = full.resolve()
    root = REPO_ROOT.resolve()
    if root not in full.parents:
        raise LarkError(f"文档必须在仓库内：{full}")
    if full.suffix.lower() != ".md":
        raise LarkError(f"只支持 Markdown 文档：{full}")
    key = full.relative_to(root).as_posix()
    raw = full.read_bytes()
    if raw.startswith(b"\xff\xfe") or raw.startswith(b"\xfe\xff"):
        content = raw.decode("utf-16")  # 兼容 PowerShell 写出的 UTF-16 文档
    elif raw.startswith(b"\xef\xbb\xbf"):
        content = raw.decode("utf-8-sig")
    else:
        content = raw.decode("utf-8")
    sha = hashlib.sha256(raw).hexdigest()
    return {"key": key, "full": full, "content": content, "sha256": sha}


def drive_index(reg):
    """返回 (canonical 文件夹内的 docx, 我的空间根目录 docx) 两个列表。"""
    idx = lark(["drive", "files", "list", "--folder-token", reg["folder_token"], "--page-all", "--format", "json"])
    folder = [f for f in (idx["data"].get("files") or []) if f.get("type") == "docx"]
    idx_root = lark(["drive", "files", "list", "--format", "json"])
    root = [f for f in (idx_root["data"].get("files") or []) if f.get("type") == "docx"]
    return folder, root


def find_by_title(title, folder_files, root_files):
    hits = [(f["token"], "folder") for f in folder_files if f.get("name") == title]
    hits += [(f["token"], "root") for f in root_files if f.get("name") == title]
    return hits


def token_meta(token):
    TMP_METAS.write_text(
        json.dumps({"request_docs": [{"doc_token": token, "doc_type": "docx"}]}, ensure_ascii=False),
        encoding="utf-8",
    )
    try:
        data = lark(["drive", "metas", "batch_query", "--data", f"@./{TMP_METAS.name}"], allow_failure=True)
        if data is None or not data.get("ok"):
            return None
        metas = (data["data"].get("metas") or [])
        return metas[0] if metas else None
    finally:
        TMP_METAS.unlink(missing_ok=True)


def assert_canonical_location(token, title, reg, dry):
    """尽量把文档归入 canonical 文件夹（标题认领兜底依赖文件夹枚举）。

    部分早期文档位于应用/群空间，move API 无法移动：此时警告并继续——
    token 已登记，就地更新不受影响；只是按标题认领的兜底对这些文档不可用。
    """
    folder_files, _ = drive_index(reg)
    if any(f["token"] == token for f in folder_files):
        return
    if dry:
        out(f"[DRY] 将移动文档«{title}»（{token}）到「{reg['folder_name']}」")
        return
    try:
        lark(["drive", "+move", "--file-token", token, "--type", "docx",
              "--folder-token", reg["folder_token"], "--format", "json"])
        out(f"[MOVE] «{title}» -> {reg['folder_name']}")
    except LarkError as exc:
        out(f"[WARN] «{title}»无法移动到「{reg['folder_name']}」（{exc}）。"
            f"已按 token 登记，sync 不受影响；按标题认领的兜底对此文档不可用，务必保留登记表条目。")


def ensure_permissions(token, title, reg, dry):
    """新建文档按仓库约定设置权限：链接组织内可读 + 群成员可编辑。"""
    if dry:
        out(f"[DRY] 将设置«{title}»权限：tenant_readable + 群可编辑")
        return
    TMP_PERM.write_text('{"link_share_entity":"tenant_readable","external_access":false}', encoding="utf-8")
    try:
        # --yes：新建文档按仓库约定统一 tenant_readable，权限形态已由仓库约定确认（见 AGENTS.md 第 5 条）
        data = lark(["drive", "permission.public", "patch", "--token", token, "--type", "docx",
                     "--data", f"@./{TMP_PERM.name}", "--yes"], allow_failure=True)
        if data is not None and data.get("ok"):
            out(f"[PERM] «{title}»：链接共享 tenant_readable")
        else:
            out(f"[WARN] «{title}»：链接共享设置失败（群成员可能打不开，请手工检查）")
        data = lark(["drive", "+member-add", "--token", token, "--type", "docx",
                     "--member-id", reg["chat_id"], "--member-type", "openchat",
                     "--perm", "edit", "--yes", "--format", "json"], allow_failure=True)
        if data is not None and data.get("ok"):
            out(f"[PERM] «{title}»：群可编辑已授予")
        else:
            out(f"[WARN] «{title}»：群可编辑授予失败（请手工检查）")
    finally:
        TMP_PERM.unlink(missing_ok=True)


def cmd_sync(args):
    dry = args.dry_run
    doc = local_doc(args.path)
    reg = load_registry()
    title = doc_title(doc["content"])
    if not title:
        raise LarkError(f"本地文档缺少 H1 标题：{doc['key']}")
    entry = reg["docs"].get(doc["key"])

    out(f"[DOC ] {doc['key']}  title=«{title}»")
    token = None
    action = None

    if entry and entry.get("doc_token"):
        meta = token_meta(entry["doc_token"])
        if meta:
            token = entry["doc_token"]
            action = "update"
            if meta.get("title") and meta["title"] != title:
                out(f"[WARN] 飞书标题«{meta['title']}»与本地«{title}»不一致，sync 后将改齐")
        else:
            out(f"[WARN] 登记的 token 已失效（可能被删除），改按标题认领：{entry['doc_token']}")

    if token is None:
        folder_files, root_files = drive_index(reg)
        hits = find_by_title(title, folder_files, root_files)
        if len(hits) > 1:
            listing = ", ".join(f"{t}({w})" for t, w in hits)
            fail(f"同题文档有 {len(hits)} 篇，先人工去重归档再 sync：{listing}")
            return
        if len(hits) == 1:
            token, where = hits[0]
            action = "adopt"
            out(f"[ADOPT] 认领既有文档 {token}（{where}）并写入登记表")
        else:
            if not args.allow_create:
                fail("未登记且飞书可见范围（canonical 文件夹/我的空间根）内无同题文档。"
                     "应用/群空间的既有文档按标题扫不到——先核对 docs/feishu-docs.json 是否丢了条目；"
                     "确认是全新文档才加 --allow-create 新建。")
                return
            if dry:
                out(f"[DRY] 将在「{reg['folder_name']}」新建文档«{title}»并登记")
                return
            TMP_MD.write_text(doc["content"], encoding="utf-8")
            try:
                data = lark(["docs", "+create", "--doc-format", "markdown", "--content", f"@./{TMP_MD.name}",
                             "--title", title, "--parent-token", reg["folder_token"], "--format", "json"])
                token = (data["data"].get("document") or {}).get("document_id")
                if not token:
                    raise LarkError("创建成功但未返回 document_id")
                action = "create"
                out(f"[CREATE] 新建文档 {token}（«{title}»）")
            finally:
                TMP_MD.unlink(missing_ok=True)
            ensure_permissions(token, title, reg, dry)

    if action in ("update", "adopt"):
        if dry:
            out(f"[DRY] 将就地更新 {token}（overwrite markdown，{len(doc['content'])} 字符）")
            return
        fetched = lark(["docs", "+fetch", "--doc", token, "--doc-format", "markdown", "--scope", "full", "--format", "json"])
        remote = (fetched["data"].get("document") or {}).get("content") or ""
        media_count = len(re.findall(r"<figure", remote))
        if media_count > 0 and not args.force:
            if action == "adopt":
                entry = reg["docs"].get(doc["key"]) or {}
                entry.update({"title": title, "doc_token": token,
                              "url": f"https://my.feishu.cn/docx/{token}",
                              "registered_at": entry.get("registered_at") or datetime.date.today().isoformat()})
                entry.setdefault("last_synced", None)
                entry.setdefault("last_synced_sha256", None)
                entry.setdefault("note", None)
                reg["docs"][doc["key"]] = entry
                save_registry(reg)
                out(f"[ADOPT] 已先登记 {token}（未更新内容：媒体守卫拦截）")
            fail(f"飞书文档含 {media_count} 个内嵌媒体块，overwrite 会丢失它们。"
                 f"先同步文字再用 `lark-cli docs +media-insert` 重嵌，或确认丢弃后加 --force。")
            return
        TMP_MD.write_text(doc["content"], encoding="utf-8")
        try:
            lark(["docs", "+update", "--command", "overwrite", "--doc", token,
                  "--doc-format", "markdown", "--content", f"@./{TMP_MD.name}", "--format", "json"])
            out(f"[SYNC ] 已就地更新 {token}（«{title}»）")
        finally:
            TMP_MD.unlink(missing_ok=True)

    if dry:
        out("[DONE] dry-run 结束，未写入任何状态")
        return

    assert_canonical_location(token, title, reg, dry)
    if action in ("update", "adopt"):
        meta = token_meta(token)
        if meta and meta.get("title") and meta["title"] != title:
            lark(["drive", "+update-title", "--token", token, "--type", "docx", "--title", title, "--format", "json"])
            out(f"[TITLE] 已改齐飞书标题为«{title}»")

    if entry is None:
        entry = {"title": title, "doc_token": token, "url": f"https://my.feishu.cn/docx/{token}",
                 "registered_at": datetime.date.today().isoformat(),
                 "last_synced": None, "last_synced_sha256": None, "note": None}
        reg["docs"][doc["key"]] = entry
    entry["doc_token"] = token
    entry["url"] = f"https://my.feishu.cn/docx/{token}"
    entry["last_synced"] = datetime.datetime.now().astimezone().isoformat(timespec="seconds")
    entry["last_synced_sha256"] = doc["sha256"]
    save_registry(reg)
    out(f"[OK  ] {action} 完成；登记表已更新：docs/feishu-docs.json")


def cmd_check(args):
    reg = load_registry()
    folder_files, root_files = drive_index(reg)

    for key, e in reg["docs"].items():
        local = REPO_ROOT / key
        if not local.exists():
            out(f"[WARN] {key} 本地文件不存在（可能尚未合入当前分支）")
            continue
        meta = token_meta(e.get("doc_token", ""))
        if meta is None:
            fail(f"{key} 登记的 token 无效：{e.get('doc_token')}")
            continue
        if meta.get("title") != e.get("title"):
            fail(f"{key} 飞书标题«{meta.get('title')}»与登记«{e.get('title')}»不一致")
        if not any(f["token"] == e["doc_token"] for f in folder_files):
            out(f"[WARN] {key} 文档不在「{reg['folder_name']}」文件夹内（应用/群空间文档无法移动；"
                f"token 已登记，就地更新不受影响，但按标题认领兜底不可用）")
        sha = hashlib.sha256(local.read_bytes()).hexdigest()
        if e.get("last_synced_sha256") and sha != e["last_synced_sha256"]:
            out(f"[STALE] {key} 本地内容自上次 sync 后有变化（{e.get('last_synced')}），记得 sync")
        out(f"[OK  ] {key} -> {e['doc_token']} «{meta.get('title')}»")

    by_name = {}
    for f in folder_files + root_files:
        by_name.setdefault(f["name"], []).append(f["token"])
    for name, tokens in by_name.items():
        if len(tokens) > 1:
            fail(f"同题文档 {name} 在飞书出现 {len(tokens)} 份：{', '.join(tokens)} —— 保留最新一份，其余归档改名")

    if EXIT["code"] == 0:
        out("[DONE] check 通过：无重复副本，登记与飞书一致")
    else:
        out("[DONE] check 发现问题，见 [FAIL] 行")


def main():
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
        sys.stderr.reconfigure(encoding="utf-8", errors="replace")
    parser = argparse.ArgumentParser(description="本地 Markdown ↔ 飞书 docx 同步（就地更新，防重复副本）")
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("check", help="体检登记表与飞书重复副本")
    p_sync = sub.add_parser("sync", help="同步一份本地 Markdown 到飞书（就地更新）")
    p_sync.add_argument("--path", required=True, help="本地 Markdown 路径（相对仓库根或绝对路径）")
    p_sync.add_argument("--allow-create", action="store_true",
                        help="确认是全新文档时才允许新建（默认拒绝新建，防重复副本）")
    p_sync.add_argument("--force", action="store_true", help="允许覆盖时丢弃飞书文档内嵌媒体")
    p_sync.add_argument("--dry-run", action="store_true", help="只打印计划，不落盘")
    args = parser.parse_args()
    try:
        {"check": cmd_check, "sync": cmd_sync}[args.command](args)
    except LarkError as exc:
        fail(str(exc))
        sys.exit(1)
    sys.exit(EXIT["code"])


if __name__ == "__main__":
    main()
