# Rollback guide

## Source rollback

The requested baseline is `63cffa17e4f71df638646ffabf6585a3e2a44489` on `codex/diagnostics-phase1`.
The baseline is also the starting commit for this work. To inspect or restore it in a separate worktree:

```powershell
git fetch origin codex/diagnostics-phase1
git worktree add ..\soccer-rollback 63cffa17e4f71df638646ffabf6585a3e2a44489
```

This retains the diagnostics branch and any local changes. To remove the temporary worktree after review,
first confirm its exact path and clean state, then run `git worktree remove ..\soccer-rollback` from the
repository. Do not reset or force-push the shared feature branch to perform a source rollback.

## Firmware rollback

There is no firmware image produced or flashed by this work. Rollback on hardware requires the exact
previously tested board-specific binary and matching MAIN/IR/MOTOR/OpenMV configuration. Restore all
compatible boards as one set; a mixed `PROTOCOL`/route build can appear as a sensor or motor failure.
Record board labels, SHA/config IDs and checksums before any deployment. If no known-good binaries are
available, do not improvise a flash: return to the bench procedure and rebuild the documented baseline.

Current `main`, `ir`, `motor`, and OpenMV source targets are listed in `FLASH_GUIDE.md`.
