---
description: Check if README is up-to-date with the current changes
agent: build
---

# README Update Workflow

Look at the current git index.

Read `README.md` thoroughly.

Your task is to go through the changes in the index, analyze them carefully, and compare them with the README.

Identify outdated bits of information in the README that have changed. Examples:
- outdated function names
- outdated function signatures
- listed limitations that are no longer true
- API changes that make example code invalid
- behavior changes that make the docs misleading

Correct those outdated parts directly in `README.md`.

Do not rewrite unrelated sections. Prefer the smallest accurate documentation fix.

Additionally, look for staged changes that introduce new capabilities or workflows that are useful to users of the library and are worth mentioning in the README.

Do not add those new sections yourself unless they are required to fix broken or misleading documentation.

Instead, report them back as suggestions for README improvements. Focus on additions that would make the README a more comprehensive and practical manual.

When finished, provide:
1. A short list of what you corrected in `README.md`
2. A separate list of suggested README additions based on new capabilities in the staged changes
3. Any uncertainty about behavior or naming that should be verified before documenting further
