# pcl_explore_human

## ROSの環境構築
下記サイトを参照  
http://wiki.ros.org/ROS/Installation

## YVT-35LXの環境構築
```
$ git clone https://github.com/at-wat/hokuyo3d.git
$ catkin build hokuyo3d
```
## 機械学習ライブラリscikit-learnおよびツールのインストール
```
$ curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py
$ python get-pip.py

# pip install scikit-learn
# pip install seaborn
# pip install matplotlib
```

## 環境構築
```
###パッケージのクローン
$ git clone https://github.com/open-rdc/pcl_explore_human.git

###依存パッケージのインストール
$ cd ~/catkin_ws/ && rosdep install --from-paths src --ignore-src -r -y

###パッケージのビルド
$ catkin build pcl_explore_human
```

## 実行例
`` $ roslaunch pcl_explore_human pcl_explore_human.launch ``

## 特徴量の抽出
- /config/pcl_extraction_human_description.yamlのパラメータを変更する
```
###pcl_extraction_human_description.yaml###

#特徴量をテキストファイルとして書き込むか tureにすると下記のパラメータの条件で特徴量をセーブ
is_save: false 

#用途に合わせてラベルの初期値を変更できる　（例）非探索対象者:0 探索対象者:1
label: 0

#用途に合わせて名前を変更できる (例)非探索対象者:description_0.csv 探索対象者:description_1.csv
description_filename:  #保存するファイル名
```

## 学習方法
- 学習データの作成・・・description_0.csv と description_1.csvを一つのファイルにまとめる　(例)description.csv
- /config/svm_trainingのパラメータを変更する
```
###svm_training.yaml###

description_filename: #学習させたい特徴量が含まれるファイル　（例）description.csv
```
- 学習させる  
```
$ rosrun pcl_explore_human svm_training.py
```

## 動作検証
- config/svm_predict.yamlのパラメータを変更する
```
###svm_predict.yaml###

is_predict: true
```
- 分類を実行する
```
$ roslaunch pcl_explore_human pcl_explore_human.launch
```

## その他 
TODO
