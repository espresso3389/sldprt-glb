# sldprt-glb 設計

SOLIDWORKS の SLDPRT / SLDASM を GLB に直接変換する C++ エンジン。元の経路は、cadmpeg（Rust、Apache-2.0）に
表示用の拡張を加えて CADIR JSON を出力し、それを Python の GLB 書き出しスクリプト（以下「参照書き出し」）で
GLB にするものだった。本エンジンはその経路のうち GLB に必要な部分を移植し、拡張も本体に組み込んでいる。

## 参照実装

| 対象 | 参照 |
| --- | --- |
| cadmpeg | `5af954216af41fdc8aa7c559d69bbd724543d134` |
| cadmpeg への拡張 | 表示色、外観の所有、アセンブリ配置、共有メッシュ、シーン表示モード、参照平面のペン色 |
| 形式仕様 | cadmpeg `docs/formats/sldprt.md` |
| Parasolid | openswx `72291dc962bbf267fa40cd57813b273108ebac53` の libopenparasolid（MIT） |

移植元のコメントには cadmpeg のファイル名・関数名（例: `decode.rs`, `graph.rs`, `fit.rs`）を記載している。

## 処理の流れ（cadmpeg `decode.rs::decode` / `decode_scan` と同順）

1. `read_container` — ネイティブ圧縮ブロック／CFB を `Section` 列へ展開（`container.cc`）。
2. `read_assembly_placements` — `swXmlContents/COMPINSTANCETREE` と DisplayLists の部品ヘッダーから
   配置・表示状態を求める（`assembly.cc`）。
3. `decode_brep` — アクティブ構成の Parasolid ストリームを解析し、B-rep・構成・面の色・面キー・
   永続 ID を `Model` に格納（`brep*.cc`）。ストリームの構文解析と解析曲面・B スプラインの読み取りは
   libopenparasolid、トポロジー・構成・属性・SP_CURVE の 3 次エルミート近似（`brep_fit.cc`）・
   ローリングボール ブレンド面の NURBS 近似（`brep_blend.cc`）は cadmpeg からの移植。
4. B-rep 由来の外観 — 面の色（`entity53#…` と `binding#face:…`）と文書の `moVisualProperties_c`
   定義（`material#…`）を `decode.rs` と同じ順に追加（`appearance.cc`）。
5. `collect_display_sections` / `push_display_meshes` — 全セクションの `uoTempFaceTessData_c` 表から
   テッセレーションを作り、表示外観（binding）を付与（`display_lists.cc`, `appearance.cc`）。
6. `assign_tessellation_owners` — 面キー → 永続 ID → 一意な曲面所有者の順で表示メッシュの
   body/faces を解決（`ownership*.cc`）。続いて表示状態（VisualStates）の面外観
   （`native-binding#…`）を適用（`brep_visual.cc`）。
7. `read_presentation` — シーン表示モード・参照平面・ペン色（`presentation.cc`）。
8. `apply_assembly` — 配置・可視性・共有メッシュ展開（`assembly.cc`）。
9. `finalize` — cadmpeg と同じく各配列を ID のバイト順で安定ソート（`engine.cc`）。
   GLB の三角形・線分の順序はこの順序に依存する。
10. `write_glb` — GLB 化、古いメッシュの検出と OCCT による再生成（`glb.cc`, `regenerate.cc`）。

各段階は `Model`（`src/model.h`、CADIR のサブセット）を受け渡す。`sldprt-glb dump` は CADIR と同じ
フィールド名の JSON を出力し、参照 CADIR と比較できる（`tools/compare_cadir.py`）。

## GLB 書き出し（`glb.cc`）

参照書き出しを関数単位で移植している（`make_glb`, `table_color`, `wire_edges`,
`reference_plane_color`, `stale_surface_bounds`, `replace_surface`）。バイト単位で一致させるため、
JSON は Python の `json.dumps(separators=(',', ':'), allow_nan=False)` と同じ規則で出力する
（`json_value.cc`: キーの挿入順、`float.__repr__` と同じ最短表現、整数と浮動小数点数の区別、
`ensure_ascii`）。色は CADIR が f32 で持つ値を JSON の倍精度へ変換した値に揃える（`f32_as_json_double`）。
参照平面は属性 ID の文字列順（`saved_reference_plane#<ordinal>:<offset>`）に並べる。

`asset.generator` と、表示モード・参照平面の情報を入れるノード extras のキーは、
`--generator` / `--extras-key` で変更できる（既定は `sldprt-glb` と `solidworksPresentation`）。

`sldprt-glb from-cadir` は cadmpeg の CADIR から GLB を作る。デコーダーと切り離して書き出し部だけを
検証するためのもので、`cadir_load.cc` が参照書き出しの読む範囲だけを読み込む。

## 古い表示メッシュの再生成（`regenerate.cc`）

単一ボディで曲面がすべて平面・円柱の部品について、B-rep の境界線の範囲と保存済みメッシュの範囲が
食い違う場合は、保存済みメッシュを古いと判定する。参照経路は B-rep を STEP に書き出し、OCCT で
テッセレーションしてから、`replace_surface` で面だけを差し替える。

本エンジンは STEP を経由せず、`Model` の B-rep から OCCT の形状を直接組み立てる
（頂点の許容差 1e-4 mm、直線・円の辺、`BRep_Builder` による面と `ShapeFix_Face`）。その後は
参照経路と同じ手順（縫合 1e-3 → `ShapeFix_Shape` → 対角長 × 0.001（最小 0.01）の弦偏差と
角度 0.5 で `BRepMesh` → XCAF 文書 → `RWGltf_CafWriter`）でテッセレーションし、`replace_surface` を
適用する。OCCT を使わずにビルドした場合、古いメッシュを検出すると終了コード 3 になる。

## エラー

構造的に不正な入力（cadmpeg が `CodecError::Malformed` を返す箇所）は `DecodeError`
（`src/errors.h`）を送出する。CLI は `solidworks_saved_image` を標準出力に書いて終了コード 3 を返す。
probe miss（その位置にレコードがない）は例外ではなく `std::nullopt`。

## ID

テッセレーション ID などは cadmpeg と同じ規則で作る（例: `sldprt:displaylist:record#<ordinal>:<index>`）。
B-rep の ID も cadmpeg と同じく Parasolid の属性番号から作る（`sldprt:brep:face#<attr>`、
別サイトは `@<site_key>` 接尾辞）。`finalize` が ID 順に並べるため、ID が一致しないと GLB の
要素順が変わる。

## 検証データ（リポジトリには含めない）

`tools/compare_all.sh` と `tools/glb_all.sh` は次の構成のディレクトリを受け取る。

- `in/<stem>.SLDPRT|.SLDASM` — 入力
- `ref/<stem>.cadir.json` — cadmpeg の CADIR
- `ref/<stem>.glb` — 参照書き出しの GLB
- `ref/timings.tsv` — 列 1 が stem、列 4 が参照の終了コード
