// 最終課題 説明資料 (docx) を生成する。PDF への変換は to_pdf.ps1 (Word)。
//   node build_report.js  → ../output/説明資料_PAI最終課題.docx
// 個人情報 (氏名・所属・メールアドレス) は課題の規定により記載しない。
const fs = require('fs');
const path = require('path');
const {
  Document, Packer, Paragraph, TextRun, HeadingLevel, AlignmentType, Table, TableRow,
  TableCell, WidthType, ShadingType, ImageRun, LevelFormat, BorderStyle, Footer, PageNumber,
} = require('docx');

const FONT = 'Yu Gothic';
const OUT = path.join(__dirname, '..', 'output', '説明資料_PAI最終課題.docx');

const runs = t => (Array.isArray(t) ? t : [t]).map(x => typeof x === 'string' ? new TextRun(x) : x);
const p = (t, opt = {}) => new Paragraph({ spacing: { after: 120, line: 320 }, ...opt, children: runs(t) });
const b = t => new TextRun({ text: t, bold: true });
const code = t => new TextRun({ text: t, font: 'Consolas', size: 19 });
const h1 = t => new Paragraph({ heading: HeadingLevel.HEADING_1, children: [new TextRun(t)] });
const h2 = t => new Paragraph({ heading: HeadingLevel.HEADING_2, children: [new TextRun(t)] });
const bullet = t => new Paragraph({ numbering: { reference: 'bul', level: 0 }, spacing: { after: 60, line: 300 }, children: runs(t) });
const num = (t, ref = 'num') => new Paragraph({ numbering: { reference: ref, level: 0 }, spacing: { after: 80, line: 300 }, children: runs(t) });
const eq = t => new Paragraph({ alignment: AlignmentType.CENTER, spacing: { before: 60, after: 120 },
  children: [new TextRun({ text: t, font: 'Cambria Math' })] });
const caption = t => new Paragraph({ alignment: AlignmentType.CENTER, spacing: { after: 220 },
  children: [new TextRun({ text: t, size: 19, color: '444444' })] });
const gap = () => new Paragraph({ spacing: { after: 100 }, children: [] });

const border = { style: BorderStyle.SINGLE, size: 4, color: '999999' };
const borders = { top: border, bottom: border, left: border, right: border };
function table(widths, rows, { hl = [] } = {}) {
  const total = widths.reduce((a, c) => a + c, 0);
  return new Table({
    width: { size: total, type: WidthType.DXA }, columnWidths: widths,
    rows: rows.map((r, ri) => new TableRow({
      tableHeader: ri === 0, cantSplit: true,
      children: r.map((c, ci) => new TableCell({
        borders, width: { size: widths[ci], type: WidthType.DXA },
        margins: { top: 50, bottom: 50, left: 90, right: 90 },
        shading: ri === 0 ? { fill: 'DCE6F0', type: ShadingType.CLEAR, color: 'auto' }
          : hl.includes(ri) ? { fill: 'FFF4D6', type: ShadingType.CLEAR, color: 'auto' } : undefined,
        children: [new Paragraph({ children: [new TextRun({ text: c, bold: ri === 0 || hl.includes(ri), size: 18 })] })],
      })),
    })),
  });
}

function pngSize(file) {
  const buf = fs.readFileSync(file);
  return [buf.readUInt32BE(16), buf.readUInt32BE(20)];
}
function img(file, w = 430) {
  const f = path.join(__dirname, 'img', file);
  const [pw, ph] = pngSize(f);
  return new Paragraph({
    alignment: AlignmentType.CENTER, keepNext: true, spacing: { before: 100, after: 40 },
    children: [new ImageRun({ type: 'png', data: fs.readFileSync(f),
      transformation: { width: w, height: Math.round(w * ph / pw) },
      altText: { title: file, description: file, name: file } })],
  });
}

// ------------------------------------------------------------------ 本文
const children = [
  new Paragraph({ alignment: AlignmentType.CENTER, spacing: { before: 200, after: 80 },
    children: [new TextRun({ text: 'Physical AI 応用編1 知能ロボティクス講座　最終課題 説明資料', size: 21, color: '555555' })] }),
  new Paragraph({ alignment: AlignmentType.CENTER, spacing: { after: 60 },
    children: [new TextRun({ text: '模倣学習で自律巡回する差動二輪ロボット', size: 34, bold: true })] }),
  new Paragraph({ alignment: AlignmentType.CENTER, spacing: { after: 300 },
    children: [new TextRun({ text: '— ROS 2 + Gazebo で「収集・変換・学習・デプロイ・DAgger」を一通り実装 —', size: 22, color: '333333' })] }),

  // 1
  h1('1. 課題の概要と目的'),
  p('倉庫や施設の見回りロボットのように、障害物を避けながら決まった地点を巡回するタスクを、ニューラルネットの方策(AI)に実行させた。ロボットは差動二輪で、使うセンサは 2D LiDAR とオドメトリ(車輪の回転から求めた位置)だけである。'),
  p('方策は、手で設計した障害物回避アルゴリズム(ポテンシャル法)を「お手本」として、その振る舞いを模倣学習(Behavior Cloning)で学ばせた。第5回講義で学んだ「ROS でデータ収集 → 学習用データへの変換 → 学習 → デプロイ → デバッグ」の流れを、講義のマニピュレータ(CRANE+ V2)ではなく移動ロボットに当てはめ、すべて自分で実装することを目的とした。'),
  p([b('タスク:'), '障害物 4 個のあるフィールドで、目標地点 A → B → C を巡回してホーム H に戻る。走行中の速度指令はすべて学習した方策が出す。']),
  p([b('AI の使い方:'), 'LiDAR と目標方向を入力して速度指令を出す多層パーセプトロン(MLP)を PyTorch で学習し、Gazebo 上のロボットで 20 Hz で推論する。']),

  // 2
  h1('2. サンプルからの発展と工夫点'),
  p('最終課題サンプル「ROS2とGazeboの連携によるロボット操作」(差動二輪 + DiffDrive プラグイン + ros_gz_bridge + キー操作)をベースにし、次のように発展させた。'),
  table([2000, 3513, 3513], [
    ['項目', 'サンプル', '本課題'],
    ['操作', '人がキーで操作', '学習した方策(AI)が自律走行'],
    ['センサ', 'なし(指令を送るだけ)', '2D LiDAR 360 本 + オドメトリ + カメラ'],
    ['環境', '平面', '壁と障害物 4 個のフィールド、目標マーカー'],
    ['AI', 'なし', '模倣学習(BC)+ DAgger、PyTorch で学習'],
    ['評価', 'なし', 'お手本と同じルートで成功率・時間・最接近距離を比較'],
    ['動画', '画面録画', 'シミュレーション時間に同期した自動録画、状態・AI の指令・お手本の指令・軌跡を重ねて表示'],
  ]),
  gap(),
  h2('工夫した点'),
  num([b('お手本を自動化して大量のデモを集めた:'), '講義では人がキー操作したデモを 10〜20 回記録したが、本課題では手で設計した回避アルゴリズムを「操作者」にし、ランダムに選んだ目標を回らせて 72 エピソード(約 20 分・333 m)を自動で記録した。']),
  num([b('ずれた状態からの立て直しも記録した(DART):'), '収集の半分では、ロボットに送る指令にだけ時間相関のあるノイズを加え、ラベルにはノイズなしのお手本の指令を使った。お手本の軌道から少しずれた状態と、そこからの戻り方も学習データに入る。']),
  num([b('データの質を管理した:'), 'お手本自身が時間切れになったエピソード(2 件)は除外した。学習用と検証用はエピソード単位で分け、同じ走行のフレームが両方に入らないようにした。']),
  num([b('失敗の原因を調べて DAgger で解決した:'), '模倣学習だけの方策は、障害物に近づきすぎると止まってしまうことがあった。原因を「学習データにない状態」と「左右どちらに回ってもよい場面の平均化」と分析し、方策自身が訪れた状態にお手本のラベルを付けて追加学習する DAgger を 1 反復行って、完走できるようにした。']),
  num([b('AI とお手本を常に比較できるようにした:'), '巡回ノードは AI が走っているときもお手本の指令を /expert_cmd として配信する。これが収集時の教師ラベル、デバッグ時の正解、動画での比較表示、DAgger のラベルを兼ねる。']),
  num([b('学習側と実行側を分けた:'), '学習は PyTorch、実行時は重みを npz に書き出して numpy だけで推論するので、ロボット側に PyTorch は不要である。観測の作り方は 1 つのモジュールにまとめ、学習時と実行時で入力がずれないようにした。']),

  // 3
  h1('3. システム構成'),
  table([2300, 6726], [
    ['項目', '内容'],
    ['環境', 'Windows 11 + WSL2(Ubuntu 22.04)、ROS 2 Humble、Gazebo Fortress(ros_gz)'],
    ['学習', 'Python 3.10、PyTorch 2(CPU)、NumPy、matplotlib'],
    ['ロボット', '0.40 × 0.30 m、2.0 kg の差動二輪(車輪半径 0.08 m、間隔 0.36 m)'],
    ['センサ', 'LiDAR 360°・360 本・0.12〜8 m・10 Hz/オドメトリ 30 Hz/搭載カメラ(表示用)'],
    ['フィールド', '8 × 8 m、箱 2 個・円柱 2 個。目標 A(赤)・B(緑)・C(黄)・H(白)'],
  ]),
  gap(),
  table([2300, 3363, 3363], [
    ['ノード', '入力', '出力'],
    ['Gazebo + ros_gz_bridge', '/cmd_vel', '/scan, /odom, /clock, カメラ画像'],
    ['patrol_node', '/scan, /odom', '/cmd_vel, /expert_cmd, /patrol/goal, /patrol/episode, /patrol/status'],
    ['video_recorder', 'カメラ画像, /odom, /cmd_vel, /expert_cmd, /patrol/status', 'mp4'],
    ['ros2 bag record(収集時)', '上記の 7 トピック', 'rosbag2(1 起動 = 1 bag)'],
  ]),
  gap(),

  // 4
  h1('4. 模倣学習の流れ'),
  h2('4.1 お手本:ポテンシャル法 + 回り込み'),
  p('目標への引力ベクトルに、LiDAR の各ビームから計算した斥力を足して進む向きを決める。距離 r のビームによる斥力の大きさは次の式で、影響距離 d₀ = 0.9 m より近い障害物だけが効く。'),
  eq('|F_rep| = k_rep · (1/r − 1/d₀) · Δθ'),
  p('障害物が目標の真正面にあると引力と斥力が打ち消し合って止まるため、空いている側へ回り込む力を加え、回る向きは障害物を抜けるまで固定する。速度は向きのずれと前方の距離に応じて落とす。'),
  h2('4.2 データ収集(rosbag2)'),
  p('ランダムに選んだ目標 12 個を回る走行を 6 回(うち 3 回はノイズあり)行い、/scan・/odom・/expert_cmd などを ros2 bag record で記録した。目標 1 つぶんを 1 エピソードとし、72 エピソード中 70 が成功した。'),
  h2('4.3 変換:観測と行動'),
  p('トピックごとに周期が違う(LiDAR 10 Hz、オドメトリ 30 Hz、指令 20 Hz)ので、講義の rosbag2lerobot と同じく、10 Hz の時刻列を作り、各時刻でそれ以前の最新の値を使って(直前値保持)そろえた。'),
  bullet([b('観測 o(39 次元):'), 'LiDAR 360 本を 10° ごと 36 区画に分けた各区画の最小距離(3 m で 1)、目標までの距離、目標方向の sin・cos']),
  bullet([b('行動 a(2 次元):'), '並進速度 / 0.35 m/s、角速度 / 1.5 rad/s(-1〜1)']),
  h2('4.4 学習とデバッグ'),
  p('MLP(39 → 256 → 256 → 2、ReLU、出力 tanh)を MSE 損失・Adam で 200 エポック学習した。データは 11,017 フレームで、56 エピソードを学習用、14 エピソードを検証用にした。検証誤差は 25 エポック付近で下げ止まり、その後は学習用の誤差だけが下がった(過学習)ため、検証誤差が最小のモデルを採用した。'),
  img('loss_curve.png', 290),
  caption('図1 学習曲線(縦軸は対数)。検証誤差は早い段階で下げ止まる'),
  p('講義の debug-policy と同様に、検証エピソードでお手本の行動と方策の推論を比べた。並進速度の平均誤差は 0.026 m/s(R² 0.73)、角速度は 0.20 rad/s(R² 0.80)だった。旋回の向きはよく追えているが、障害物の手前でお手本が減速する場面では、方策の減速が弱い(図2 の 15〜18 秒)。'),
  img('debug_policy.png', 330),
  caption('図2 検証エピソードでのお手本(実線)と方策(破線)の比較'),
  h2('4.5 失敗の分析と DAgger'),
  p('この方策を評価ルートで 3 回走らせると、1 回は C → H の区間で円柱に接したまま動けなくなった(図3)。このとき方策は並進 0・旋回 0 を出力していたが、お手本なら 1.5 rad/s で旋回する場面だった。'),
  img('bc_stuck.png', 360),
  caption('図3 DAgger 前の方策が円柱の横で停止した場面。お手本なら旋回する(画面左上の比較表示)'),
  p('原因は二つ考えられる。一つは、お手本は障害物から 0.4 m 以上離れて走るため「障害物にほぼ接した状態」が学習データにないこと(共変量シフト)。もう一つは、障害物の目の前では左右どちらに回ってもよく、MSE で学習すると二つの答えが平均されて「回らない」になること(多峰性)である。'),
  p('そこで DAgger を 1 反復行った。学習した方策にランダムな目標 10 個を回らせる走行を 4 回行い、方策が実際に訪れた状態でのお手本の指令をラベルとして記録した。元のデータと合わせた 110 エピソード・16,647 フレームで学習し直した。'),

  // 5
  h1('5. 結果'),
  p('評価ルート A → B → C → H を、録画なしの同じ条件で走らせた結果を示す。'),
  table([2300, 1500, 1800, 1500, 1926], [
    ['制御器', '完走', '所要時間', '走行距離', '最接近距離'],
    ['お手本(ポテンシャル法)', '1 / 1 回', '69.3 s', '20.3 m', '0.45 m'],
    ['模倣学習(BC)のみ', '2 / 3 回', '71.7〜100.7 s', '17.5〜20.4 m', '0.20〜0.23 m'],
    ['BC + DAgger(提出動画)', '3 / 3 回', '69.2〜71.6 s', '19.9〜20.5 m', '0.22〜0.24 m'],
  ], { hl: [3] }),
  p('※ 最接近距離は LiDAR(ロボット中心)から障害物までの最小距離。車体の端までは約 0.2 m。お手本は決定的に動くため 1 回のみ計測した。', { spacing: { before: 60, after: 160 } }),
  p('DAgger 後の方策は 3 回ともすべての地点に到達し、所要時間もお手本とほぼ同じになった。提出動画(75 秒)も DAgger 後の方策の走行で、69.8 秒で完走している。'),
  img('ai30.png', 400),
  caption('図4 提出動画(30 秒時点)。AI が B へ向かっている。左上に AI の指令とお手本の指令を並べて表示'),
  img('ai74.png', 400),
  caption('図5 提出動画の最後。全地点を回り、軌跡マップに一周の経路が残っている'),

  // 6
  h1('6. 考察と今後の課題'),
  bullet([b('模倣学習の典型的な失敗を再現・解決できた:'), '検証データ上の誤差は小さくても、実際に走らせると学習データにない状態で失敗する。誤差の数値だけでなく、デプロイして動きを見ることが重要だと実感した。DAgger はその状態を直接データに加えるので効果が大きかった。']),
  bullet([b('障害物との距離が近い:'), 'DAgger 後もお手本より障害物に近づく(0.45 m に対して 0.22 m)。減速のラベルを重視する損失の重み付けや、障害物との距離が近いフレームを多く学習させることで改善できると考える。']),
  bullet([b('多峰性への対応:'), '左右どちらに避けるかのように答えが複数ある行動は、MSE の回帰では平均化される。講義で紹介された ACT や Diffusion Policy のように、行動の分布を表せる方策を使うと根本的に改善できる。']),
  bullet([b('評価の回数が少ない:'), 'シミュレーションの実行に時間がかかり、各 3 回しか評価していない。シミュレーションは毎回わずかに結果が変わるため、回数を増やして成功率を見積もる必要がある。']),
  bullet([b('実機への応用:'), '入力は LiDAR とオドメトリだけなので、同じ構成の実機ロボットにも移しやすい。ただし実機では車輪の滑りでオドメトリに誤差がたまるため、自己位置推定と組み合わせる必要がある。']),
  bullet([b('実行環境の工夫:'), 'WSL2 では Gazebo の既定の描画エンジンが異常終了したため、旧版の描画エンジンとソフトウェア描画に切り替えた。また PC のメモリが少なく、長い処理が途中で止まったので、収集を 10 分以内の単位に分けて実行した(第3回講義のメモリ・OOM の話題と同じ問題である)。']),

  // 7
  h1('7. 提出物と再現方法'),
  p('コード一式は GitHub リポジトリで公開している(MIT ライセンス)。主なファイルと、最初から再現する手順は次のとおり。'),
  table([3800, 5226], [
    ['ファイル', '内容'],
    ['diffbot_task/patrol_node.py', '巡回ノード(お手本 / 学習した方策)'],
    ['diffbot_task/features.py', '観測・行動の定義、numpy での推論'],
    ['diffbot_task/rosbag2dataset.py', 'rosbag2 → 学習用データ(10 Hz、直前値保持)'],
    ['diffbot_task/train_bc.py / debug_policy.py', '模倣学習 / 方策のデバッグ'],
    ['launch/collect.launch.py', 'デモ収集・DAgger 収集(rosbag2)'],
    ['launch/patrol.launch.py', '巡回の実行と録画'],
    ['setup/*.sh', '環境構築・一括実行・DAgger・評価・録画'],
    ['output/ai_results/', '学習済みの重み、学習曲線、デバッグ・評価結果'],
  ]),
  gap(),
  p([code('sudo bash setup/install_ros2_humble.sh && sudo bash setup/install_ai_deps.sh')], { spacing: { after: 30 } }),
  p([code('bash setup/run_pipeline.sh                # 収集→変換→学習→デバッグ→評価')], { spacing: { after: 30 } }),
  p([code('bash setup/run_dagger.sh collect 0 1 2 3 && bash setup/run_dagger.sh train')], { spacing: { after: 30 } }),
  p([code('bash setup/record_ai_demo.sh              # DAgger 後の方策の走行を録画')], { spacing: { after: 200 } }),

  h1('参考文献'),
  bullet('最終課題サンプル「ROS2とGazeboの連携によるロボット操作」https://qiita.com/N622/items/9d89f77d85d9da0af29e'),
  bullet('Physical AI 応用編1 第5回 講義資料「ROS によるデータ収集と LeRobot を用いた模倣学習」'),
  bullet('S. Ross, G. Gordon, J. A. Bagnell, "A Reduction of Imitation Learning and Structured Prediction to No-Regret Online Learning," AISTATS, 2011.'),
  bullet('M. Laskey et al., "DART: Noise Injection for Robust Imitation Learning," CoRL, 2017.'),
  bullet('O. Khatib, "Real-Time Obstacle Avoidance for Manipulators and Mobile Robots," IJRR, 1986.'),
];

const doc = new Document({
  styles: {
    default: { document: { run: { font: FONT, size: 20 } } },
    paragraphStyles: [
      { id: 'Heading1', name: 'Heading 1', basedOn: 'Normal', next: 'Normal', quickFormat: true,
        run: { size: 26, bold: true, color: '1F3864', font: FONT },
        paragraph: { spacing: { before: 300, after: 140 }, outlineLevel: 0, keepNext: true, keepLines: true,
          border: { bottom: { style: BorderStyle.SINGLE, size: 6, color: '1F3864', space: 2 } } } },
      { id: 'Heading2', name: 'Heading 2', basedOn: 'Normal', next: 'Normal', quickFormat: true,
        run: { size: 22, bold: true, color: '2E5597', font: FONT },
        paragraph: { spacing: { before: 200, after: 80 }, outlineLevel: 1, keepNext: true, keepLines: true } },
    ],
  },
  numbering: { config: [
    { reference: 'bul', levels: [{ level: 0, format: LevelFormat.BULLET, text: '・', alignment: AlignmentType.LEFT,
      style: { paragraph: { indent: { left: 480, hanging: 300 } } } }] },
    { reference: 'num', levels: [{ level: 0, format: LevelFormat.DECIMAL, text: '(%1)', alignment: AlignmentType.LEFT,
      style: { paragraph: { indent: { left: 520, hanging: 420 } } } }] },
  ] },
  sections: [{
    properties: { page: { size: { width: 11906, height: 16838 },
      margin: { top: 1250, bottom: 1250, left: 1300, right: 1300 } } },
    footers: { default: new Footer({ children: [new Paragraph({ alignment: AlignmentType.CENTER,
      children: [new TextRun({ children: ['- ', PageNumber.CURRENT, ' -'], size: 18 })] })] }) },
    children,
  }],
});

Packer.toBuffer(doc).then(buf => {
  fs.writeFileSync(OUT, buf);
  console.log('saved:', OUT);
});
