# sldprt-glb

SOLIDWORKS の部品（`.SLDPRT`）・アセンブリ（`.SLDASM`）を、保存済みの表示メッシュから GLB に変換する
C++ エンジンです。「cadmpeg（Rust）→ CADIR JSON → Python の GLB 書き出し」という経路を、
1 つの実行ファイルに置き換えるものです。

- 検証データ 73 件のうち 72 件で、元の経路と同じ GLB をバイト単位で出力します。
  処理時間は元の経路の約 1/16、クリーンビルドは約 35 分から 24 秒になりました
  （詳細は [docs/RESULTS.md](docs/RESULTS.md)）。
- 表示モード、参照平面のペン色、アセンブリの配置と共有メッシュ、未対応の境界線があってもプレビューを残す処理、
  古い表示メッシュの検出と B-rep からの再生成を含みます。
- 古い表示メッシュの再生成は、STEP を経由せずに B-rep から OpenCASCADE の形状を直接組み立てて行います。

設計の詳細は [docs/DESIGN.md](docs/DESIGN.md) を参照してください。

## ビルド

必要なもの: C++23 コンパイラー（GCC 12 以降）、CMake 3.24 以降、zlib、
OpenCASCADE 7.6 以降（再生成を使う場合）。GCC 12 + OpenCASCADE 7.6（Debian 12）と
GCC 15 + OpenCASCADE 7.9 で、同じ GLB を出力することを確認しています。Parasolid の読み取りには openswx の libopenparasolid（MIT）を
CMake の FetchContent で取得します。

```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
```

```bash
cmake --build build
```

Debian/Ubuntu では `libocct-*-dev`（`libocct-data-exchange-dev` など）と `zlib1g-dev` を入れてください。
OpenCASCADE を使わない場合は `-DSWGLB_WITH_OCCT=OFF` を付けます。この場合、古い表示メッシュが
検出されると変換は失敗扱い（終了コード 3）になります。

## 使い方

```bash
sldprt-glb input.SLDPRT output.glb
```

| 結果 | 終了コード | 標準出力 |
| --- | --- | --- |
| 変換成功 | 0 | なし |
| 表示データがない、または解析できない | 3 | `solidworks_saved_image` |
| 引数の誤り、出力先が既に存在する | 2 | なし（標準エラーに理由） |

環境変数 `SWGLB_DEBUG=1` を設定すると、終了コード 3 の理由を標準エラーに出力します。

| オプション | 内容 |
| --- | --- |
| `--generator <text>` | GLB の `asset.generator`（既定 `sldprt-glb`） |
| `--extras-key <key>` | 表示モード・参照平面の情報を入れるノード extras のキー（既定 `solidworksPresentation`） |

検証用のサブコマンド:

| コマンド | 内容 |
| --- | --- |
| `sldprt-glb dump <input> [--no-mesh] [--no-brep]` | 解析結果を CADIR と同じフィールド名の JSON で出力 |
| `sldprt-glb sections <input>` | コンテナーのセクション一覧 |
| `sldprt-glb from-cadir [options] <cadir.json> <out.glb> [--part]` | cadmpeg の CADIR から GLB を作成（GLB 書き出し部の単体検証） |
| `sldprt-glb brep-glb <input> <out.glb>` | 再生成用に B-rep をテッセレーションした中間 GLB |

## 検証ツール

- `tools/compare_cadir.py`, `tools/compare_all.sh` — `dump` の出力と cadmpeg の CADIR を領域ごとに比較
- `tools/compare_glb.py`, `tools/glb_all.sh` — GLB と終了コードを参照の結果と比較

検証データはリポジトリには含めません。

## ライセンス

Apache-2.0。cadmpeg（Apache-2.0）からの移植部分と openswx（MIT）の利用については [NOTICE](NOTICE) を参照してください。
