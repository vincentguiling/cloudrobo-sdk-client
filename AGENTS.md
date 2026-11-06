# AGENTS.md

面向在本仓库工作的自动化代理与贡献者的硬约定。

## 文档事实源

- 调研文档放 `docs/research/`，设计文档放 `docs/design/`，均为 Markdown；**仓库是唯一事实来源**，飞书文档只是同步副本。

## 飞书同步（防止重复出新副本）

1. 需要同步到飞书时**一律**用：

   ```bash
   python tools/feishu_doc_sync.py sync --path <md路径>
   ```

   它按 `docs/feishu-docs.json` 登记的固定 token **就地更新**；token 失效时按标题在飞书目录内认领既有文档；确认不存在才新建并自动登记。
2. **禁止**直接用 `lark-cli docs +create`（或 import）上传调研/设计文档——每执行一次就多出一个新副本。
3. 飞书目录固定为「CloudRobo 调研与设计」（folder token `Dl3sfLpSGlQjfxd31oScQJkKnse`）；飞书文档标题 = 本地文档 H1 标题。
4. 本地文档内容有修订后，重跑一次 sync 更新同一篇；改完可跑 `python tools/feishu_doc_sync.py check` 体检（同题多份会直接报 FAIL，先归档去重再 sync）。注意：飞书文档内嵌了图片/视频时，sync 默认拒绝覆盖（防止丢媒体）——先用 `lark-cli docs +media-insert` 安排重嵌，或确认丢弃后加 `--force`。
5. 新建文档的权限由工具自动设置：链接组织内可读（tenant_readable）+ 群 `oc_9193635ab43cf0c028434d11558ff25f` 可编辑；已有文档的权限不在 sync 时改动。
6. `docs/feishu-docs.json` 的条目是固定文档标识：不要删除或改写 doc_token；文档位于应用/群空间无法归入文件夹时，登记表条目是唯一的认领依据。
7. 不得上传密钥、凭据、私有 URL 或未脱敏的客户数据。
