---
name: readme-check
description: Check whether README.md is accurate and current relative to staged changes, correcting outdated documentation and reporting useful additions.
---

# README Update Workflow

Use this skill when reviewing documentation against staged repository changes.

1. Inspect the current git index and identify all staged changes.
2. Read `README.md` thoroughly.
3. Compare the staged changes with the README and identify outdated information, including:
   - outdated function names
   - outdated function signatures
   - listed limitations that are no longer true
   - API changes that make example code invalid
   - behavior changes that make the documentation misleading
4. Correct outdated parts directly in `README.md`.

Do not rewrite unrelated sections. Prefer the smallest accurate documentation fix.

Also look for staged changes that introduce new capabilities or workflows useful to library users and worth mentioning in the README. Do not add these sections yourself unless they are required to fix broken or misleading documentation. Report them as suggestions instead, focusing on additions that would make the README a more comprehensive and practical manual.

When finished, report:

1. A short list of corrections made to `README.md`.
2. A separate list of suggested README additions based on new capabilities in the staged changes.
3. Any uncertainty about behavior or naming that should be verified before documenting further.
