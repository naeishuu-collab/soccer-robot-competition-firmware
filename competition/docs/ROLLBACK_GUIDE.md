# 復元ガイド

競技用ブランチを維持したまま、旧版を別の作業フォルダに展開できる。ブランチ作成時のローカル復元参照は `archive/pre-rebuild-main` と `archive/diagnostics-phase1`。SHAは以下のとおり。

```powershell
git worktree add --detach ../soccer-rollback-main 44cd80b3fd2a5137721e3f97bdd7603d21474557
git worktree add --detach ../soccer-rollback-diagnostics 968bc520a1894ae3f871e5d553089600b20f30fc
```

旧mainとDraft PR #6はGitHubからも取得可。各版のREADMEから書き込み対象と設定を確認する。電源を切ってからMAIN、MOTOR、IRの対応する旧版をセットで復元し、OpenMVは作業前に保存したスクリプトと現物用の校正を復元する。最後に車輪を浮かせて通信・出力・停止を確認する。

ZIP原本の比較テストには元のアーカイブを使う。元ファイルのハッシュは [構成・通信・監査](ARCHITECTURE.md) に記載。
