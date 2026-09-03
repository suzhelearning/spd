# Save Markdown Skill Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Install and verify a global `save-md` skill that turns generated knowledge into project-scoped Obsidian Markdown when the user says `save md`.

**Architecture:** A single model-invoked `/home/current/.agents/skills/save-md/SKILL.md` owns trigger, content-shaping, project-path, filename, write, and success-report contracts. It uses existing shell and file-writing tools; no helper script, service, or template file is added.

**Tech Stack:** Agent Skills Markdown frontmatter, Pi/OMP tools, UTF-8 Markdown, Obsidian YAML frontmatter.

**Spec:** `docs/superpowers/specs/2026-09-03-save-md-skill-design.md`

## Global Constraints

- Obsidian root is exactly `/home/current/Documents/obsidian`.
- Prefer Git root basename as project name; otherwise use current working-directory basename.
- Save distilled knowledge, plans, procedures, or decisions rather than a raw chat transcript.
- Existing notes are never overwritten.
- A successful response reports only the absolute saved path.
- The installed skill contains no helper script, service, template file, automatic index, or Obsidian configuration.

---

### Task 1: Establish Baseline Failures

**Files:**
- Read: `docs/superpowers/specs/2026-09-03-save-md-skill-design.md`
- Do not create the production skill yet.

**Interfaces:**
- Consumes: three `save md` pressure scenarios.
- Produces: observed baseline failure patterns that the skill must correct.

- [ ] **Step 1: Run standalone-trigger baseline**

Give a fresh agent an existing technical answer followed only by `save md`. Instruct it not to inspect or load a `save-md` skill and not to mutate files. Capture whether it identifies the required vault, project folder, knowledge distillation, and response contract.

- [ ] **Step 2: Run appended-trigger baseline**

Give a fresh agent `Explain a technical topic and save md`. Capture whether it proposes generating the answer first and saving the distilled result under the current project.

- [ ] **Step 3: Run pressure baseline**

Tell a fresh agent to save quickly despite a same-name note and no explicit filename. Capture whether it overwrites, asks unnecessary questions, stores raw transcript, or chooses another directory.

- [ ] **Step 4: Classify failures**

Map every observed failure to one required positive recipe in `SKILL.md`: trigger scope, saved-content shape, project resolution, collision behavior, error behavior, or final response.

---

### Task 2: Write the Minimal Skill

**Files:**
- Create: `/home/current/.agents/skills/save-md/SKILL.md`

**Interfaces:**
- Consumes: the approved specification and Task 1 failure patterns.
- Produces: a model-invoked skill named `save-md`.

- [ ] **Step 1: Write valid frontmatter**

Use exactly:

```yaml
---
name: save-md
description: Use when the user says "save md" or explicitly asks to save generated knowledge, plans, procedures, or decisions to Obsidian.
---
```

- [ ] **Step 2: Write the positive execution recipe**

The body must require this order:

```text
select generated knowledge → distill it → resolve project → create directory
→ choose collision-free timestamped filename → write UTF-8 note → report absolute path
```

Define standalone and appended triggers, allowed `type` values, YAML frontmatter, filename sanitization, no-overwrite behavior, and explicit empty/write-failure responses.

- [ ] **Step 3: Keep the skill self-contained**

Keep all required behavior in `SKILL.md`, under 500 words if clarity permits. Do not add supporting files.

- [ ] **Step 4: Validate static structure**

Check that the installed path exists, frontmatter parses, `name` contains only lowercase letters and a hyphen, the description begins with `Use when`, and the body contains the exact Obsidian root.

---

### Task 3: Verify Skill Behavior

**Files:**
- Read: `/home/current/.agents/skills/save-md/SKILL.md`
- Create during behavior smoke: `/home/current/Documents/obsidian/spd/<timestamp>-<title>.md`

**Interfaces:**
- Consumes: the installed skill and the same scenarios from Task 1.
- Produces: evidence that agents follow the save contract and one real Obsidian smoke-test note.

- [ ] **Step 1: Re-run scenarios with the skill**

Give fresh agents the installed `SKILL.md` plus the same standalone, appended, and collision pressure scenarios. Require each to return the proposed note body, destination, and final response. Verify each baseline failure is corrected.

- [ ] **Step 2: Run a real save smoke test**

Invoke the installed skill on a short technical knowledge sample from `/home/current/syz/spd`. Confirm it creates a UTF-8 Markdown file under `/home/current/Documents/obsidian/spd/`.

- [ ] **Step 3: Inspect observable output**

Confirm the note has YAML `title`, ISO local `date`, `project: spd`, one allowed `type`, an H1 title, distilled content, and no raw conversation transcript.

- [ ] **Step 4: Verify no-overwrite behavior**

Exercise a filename collision and confirm the existing file remains byte-identical while the new path receives a numeric suffix.

- [ ] **Step 5: Report completion**

Report the installed skill path, behavior evidence, and created smoke-test note path. Do not claim autonomous discovery beyond the tested fresh-agent context.
