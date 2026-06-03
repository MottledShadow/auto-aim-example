## Code style

Prefer minimal, idiomatic code over defensive or highly abstract code.

Do not add helpers, wrappers, classes, schemas, fallback branches, or comments unless they are necessary for this exact change.

Preserve existing project style. Avoid introducing new patterns.

Before editing, identify the smallest viable diff.

After editing, remove any code that is only theoretically useful.

When adding a branch, guard, abstraction, or dependency, briefly justify why it is necessary.