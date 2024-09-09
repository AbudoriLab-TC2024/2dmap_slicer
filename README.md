# 2dmap_slicer
PointCloudからROS 2のNav2で使用できる2DMapを出力します。

## 目次

## 概要
- SLAMして得られたPointCloudからNavigation2の2DMapとして使用できるように、PointCloudを加工しpgmファイルを出力するプロジェクトです。
- 大まかな手順として、最も大きな地面の推定を行い、傾きを水平に補正します。

## ファイル構造
```tree


```

## 依存ライブラリ
このリポジトリは以下のライブラリに依存します。

- PointCloudLibrary 1.8
- OpenCV

OpenCVはpgmファイルの書き出しにimwriteに使うのみです。ほとんどのバージョンで利用できるはずです。

## インストール
まずはRequirementのライブラリをインストールします。
環境はUbuntu24.04を想定しておりますが、Ubuntuで上記のライブラリをインストールできれば、利用可能です。

以下のように、各ライブラリをaptでインストールします。ライブラリのインストールができれば、コードを各自環境にクローンしてください。
### PCLのインストール
```
sudo apt update
sudo apt install libpcl-dev
```
### OpenCVのインストール
```
sudo apt update
sudo apt install libopencv-dev
```
### コードのクローン
```
git clone https://github.com/AbudoriLab-TC2024/2dmap_slicer.git
```

## 使用方法
### ビルド
CMakeFile.txtを使用してビルドします。以下のコマンドを入力します。
```
cd 2dmap_slicer
mkdir build
cd build
cmake ..
make
```

buildディレクトリに各実行ファイルが作成されます。各プログラムの使用方法は後述の詳細説明をご覧ください。

### view_cloud

### rotate_cloud

### remove_ground

### others...





### Sample Point Cloud
SLAMアルゴリズムから得られた点群を`input_cloud`ディレクトリに入れておくと便利です。

また、サンプル用にこの開発で動作確認した点群は以下のURLからダウンロードできます。自己責任でご使用ください。

https://drive.google.com/file/d/10XxAIzNWsZk2X0db_2p7WuZvbXl6Vrf2/view?usp=sharing

### Output Cloud
このコードの出力結果はこの`output_cloud`ディレクトリに出力します。
このディレクトリを消去してしまうと、時間をかけて出力した点群を保存せず破棄してプログラムを終了します。時間の無駄ですね。

このプログラムが出力する点群は以下のREADMEをご覧ください。

ここにREADMEのリンクを貼る。


## ライセンス
このコードはApatch2.0ライセンスで公開しておりますが、使用する場合は依存ライブラリのライセンスに従ってください。

テスト用の点群は自己責任での使用をお願いします。

## 作者
Abudori Lab. abudori

[Twitter](https://x.com/abudori_imgproc)

自律走行ロボットについてブログを書いています。このプロジェクトは、つくばチャレンジ2024で開発したコードを公開しております。詳細はブログをご覧ください。

[Blog](https://www.abudorilab.com/)

解説記事

[記事1](https://www.abudorilab.com/)
