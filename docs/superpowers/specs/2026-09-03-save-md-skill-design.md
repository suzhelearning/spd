# `save-md` Skill 设计

## 目标

创建一个全局、模型可调用的 `save-md` skill。用户消息包含独立短语 `save md` 时，助手把当前产生的知识、方案、计划、操作步骤或结论整理成可长期查阅的 Obsidian Markdown 笔记。

固定 Obsidian 根目录：

```text
/home/current/Documents/obsidian
```

笔记按当前项目分目录保存。当前仓库对应：

```text
/home/current/Documents/obsidian/spd/
```

## 调用合同

以下两种调用均触发 skill：

```text
save md
解释 222M diffusion policy 的训练方法，然后 save md
```

- 单独发送 `save md`：整理并保存上一条有实质内容的助手回答。
- 附加在任务消息中：先完成该任务，再把本次生成内容整理并保存。
- `save md` 是明确触发词；普通的 Markdown 或 Obsidian 讨论不触发保存。
- 没有可沉淀内容时不创建空文件，并明确返回“没有可保存的知识内容”。

## Skill 形态

安装为一个 model-invoked skill：

```text
/home/current/.agents/skills/save-md/SKILL.md
```

只创建 `SKILL.md`，不增加 Python helper、模板文件或后台服务。现有目录创建和文件写入工具足以完成操作。

Frontmatter：

```yaml
---
name: save-md
description: Use when the user says "save md" to preserve generated knowledge, plans, procedures, or conclusions as an Obsidian Markdown note.
---
```

## 项目目录解析

1. 当前目录位于 Git 工作树时，使用 Git 根目录 basename。
2. 不在 Git 工作树时，使用当前工作目录 basename。
3. 将路径分隔符和非法文件名字符替换为 `-`。
4. 在 `/home/current/Documents/obsidian/<project>/` 下创建笔记。

项目解析失败时，不写入未知或根级目录；返回具体错误。

## 笔记内容

保存的是整理后的知识文档，不是聊天转录。

保留：

- 主题和结论；
- 必要背景与约束；
- 分步骤说明；
- 代码、命令、公式和表格；
- 风险、边界和验证结果；
- 计划中的阶段、任务和 checklist。

去除：

- 寒暄；
- “你刚才问了”等对话元叙述；
- 重复内容；
- 工具调用过程；
- 与知识主题无关的会话状态。

不得补写未经当前回答或可靠上下文支持的事实。原内容存在不确定性时，在笔记中保留该不确定性。

## 文件格式

文件名：

```text
YYYY-MM-DD-HHmmss-可读标题.md
```

标题从内容主题提取，清理 `/\\:*?"<>|` 和控制字符。若目标文件已存在，追加递增后缀 `-2`、`-3`，不覆盖旧文件。

正文格式：

```yaml
---
title: 笔记标题
date: 2026-09-03T14:30:00
project: spd
type: knowledge
---

# 笔记标题

正文
```

`type` 按主要内容取以下一个值：

- `knowledge`
- `plan`
- `procedure`
- `decision`

一篇笔记只有一个主类型，避免引入额外分类体系。

## 数据流

```text
用户触发 save md
  → 确定保存范围
  → 提取主题并整理正文
  → 解析项目名
  → 创建项目目录
  → 生成无冲突文件名
  → 写入 UTF-8 Markdown
  → 返回绝对路径
```

成功后只报告：

```text
已保存：/home/current/Documents/obsidian/<project>/<file>.md
```

## 错误处理

- 目标根目录不可创建或不可写：不降级到其他目录，返回失败路径和系统错误。
- 项目名无法安全解析：停止，不写入根目录。
- 内容为空：不创建文件。
- 文件名冲突：追加数字后缀，不覆盖。
- 写入失败：不声称保存成功。

## 验证

采用 skill TDD：

1. 在未加载 skill 的基线场景中测试 `save md`，记录未整理、路径错误或未写文件等失败。
2. 创建最小 `SKILL.md` 后重复同一场景。
3. 验证单独触发和附加触发。
4. 验证当前项目生成 `/home/current/Documents/obsidian/spd/`。
5. 验证笔记不是聊天原文，且代码、步骤和 checklist 得以保留。
6. 验证同名冲突不覆盖旧文件。
7. 验证空内容和不可写目录不会产生虚假的成功响应。

## 非目标

- 自动保存每一条回答；
- 保存完整对话历史；
- 修改 Obsidian 配置或插件；
- 自动生成双向链接、MOC、标签树或索引；
- 同步、提交或发布 Obsidian vault；
- 为简单文件写入增加常驻服务或脚本。
