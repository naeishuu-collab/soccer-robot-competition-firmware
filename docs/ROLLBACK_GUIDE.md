# Rollback guide

## Source rollback

The saved baseline is `44cd80b3fd2a5137721e3f97bdd7603d21474557`. In a separate checkout/worktree:

```powershell
git fetch origin main
git worktree add ..\soccer-rollback 44cd80b3fd2a5137721e3f97bdd7603d21474557
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
