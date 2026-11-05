# AGENTS.md

本文件记录本仓库面向自动化编码代理的约定。

## 文档交付规范

1. **仓库是事实来源**：调研文档放 `docs/research/`，设计文档放 `docs/design/`，均为 Markdown。
2. **调研、设计类文档除提交仓库外，必须同步上传到飞书**，并在回复中给出飞书文档链接。
   - 飞书目录：**「CloudRobo 调研与设计」**（Drive folder token `Dl3sfLpSGlQjfxd31oScQJkKnse`）
   - 上传：先把待上传内容写到仓库根目录下的临时 `.md`（`--content` 的 `@file` 只接受当前工作目录的相对路径），再执行

     ```bash
     lark-cli docs +create --doc-format markdown --content "@./<tmp>.md" \
       --title "<标题>" --parent-token Dl3sfLpSGlQjfxd31oScQJkKnse
     ```

     上传后删除临时文件。
   - 创建后设置共享：`link_share_entity=tenant_readable`、`external_access=false`，否则群成员打不开。

     ```bash
     lark-cli drive permission.public patch --token <docx_token> --type docx \
       --data "@./perm.json" --yes    # {"link_share_entity":"tenant_readable","external_access":false}
     ```

3. **标题对齐**：飞书文档标题与仓库文件主题一致（可省略仓库 Markdown 的首个 H1，避免重复标题）。
4. **不得上传**密钥、凭据、私有 URL 或未脱敏的客户数据；飞书文档按组织内可见处理，不代表已脱敏。
