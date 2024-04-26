cnoid_rover_pkgs
=======

概要
=======
このパッケージは[Vston社メガローバー](https://github.com/vstoneofficial/megarover_samples)を[Choreonoid](https://github.com/choreonoid/choreonoid)上で動作させるためのモデルデータやコントローラー、SLAM、ナビゲーション設定等が含まれてます。

インストール方法
=======
### 1．ROSワークスペースのディレクトリに移動し、リポジトリをクローン
```bash 
cd ~/{ROSワークスペースディレクトリ}/src/
git clone -b "2023年度成果物" https://github.com/jadsys/cnoid_rover_pkgs.git
```
### 2．Buildを行う
```bash 
cd cnoid_rover_pkgs
catkin build 
```
### X. 依存関係の解決
当パッケージでは外部パッケージとして以下を利用しております。
- [Choreonoid](https://github.com/choreonoid/choreonoid.git)
- [Choreonoid_ros](https://github.com/choreonoid/choreonoid.git)

それぞれのインストール手法については、Choreonoidの[公式ドキュメント](https://choreonoid.org/ja/manuals/latest/ros/build-choreonoid.html#ros-choreonoid-add-package-sources)を参照して下さい。

ライセンス
=======
## BSD 3-Clause License

Copyright (c) 2023, Japan Advanced System,Ltd.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its contributors 
   may be used to endorse or promote products derived from this software 
   without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

* * *
## 使用ライブラリ関係
### [Choreonoid](https://github.com/choreonoid/choreonoid.git)
 [MIT License](https://opensource.org/licenses/MIT): 
 Copyright (c) 2019-2024 Choreonoid Inc.
 Copyright (c) 2007-2019
    Shin'ichiro Nakaoka and the Choreonoid development team,
    Intelligent Systems Research Institute,
    National Institute of Advanced Industrial Science and Technology (AIST)

### [Choreonoid_ros](https://github.com/choreonoid/choreonoid.git)
 [MIT License](https://opensource.org/licenses/MIT): 
