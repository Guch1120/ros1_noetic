#用途
ros2 humble環境のdockerコンテナであり，rviz・gazebo・flexbeをデフォルトで使用可能状態にしている．
Terminaotr上でエンターキーを押すと起動し，Ctrl+Cで終了する．終了後再び待機状態となりエンターキーを再度押すと起動する．
右下のTerminatorは何も起動していないただのTeminaotrなので任意のコマンドを実行可能．


# 使い方
###  1.`ls`でdocker-compose.yamlと同じディレクトリであることを確認する．違うときは適宜`cd`で移動．    
###  2.```docker-compose up --build```でdockerのイメージをビルドしてコンテナ作成（up）させる．  
- 完了すると作成したコンテナに入るようになっている．
- 実行するとTerminatorが起動し4つのウィンドウが存在する．
###  3.右下の Terminatorで`bash script/setup_flexbe.sh`を実行しflexbeの環境を構築する，  
- 内容はflexbeのリポジトリのクローン・rosdepの初期化とアップデート・colcon build．
###  4.終了するとTerminator上でも指示が表示されるとおり，一度 Terminatorを左上の×印で閉じる．    
- これで環境構築は完了．  
- 以降はrun_docker.bashと同じディレクトリで
 ```
./run_docker.bash
```
で実行するとコンテナの起動とコンテナへのアクセス・Terminatorの起動までを行える．


# エラーとその解決法備忘録集

##　gitにsshkeyを保存するとき、"ssh-rsa ......="のとこまでコピペする。
ssh-rsaの後には半角スペースがあるがそのままで問題ない。
"ssh-rsa"の部分が抜けていたり、"="が抜けているとエラーとなる。


### エラー名
```
Error: unable to open display
```

### 解決するために行ったコマンド
```bash
xhost +local:dockeruser
```

### エラー解説と解決方法の解説
- **エラー解説**: DockerコンテナからX11を利用してGUIアプリケーションを実行する場合、ホストのディスプレイを利用する設定が必要です。このエラーは、`roscore`を実行しようとした際に、X11ディスプレイの設定が適切でなかったために発生しました。
- **解決方法**: `xhost +local:dockeruser`を実行して、ローカルユーザーがX11ディスプレイにアクセスできるように許可します。その後、コンテナ内でGUIアプリケーションを実行できます。

---
---

# 初心者向け解説

## X11ディスプレイの設定について
- **X11ディスプレイ**: X11は、LinuxなどのUnix系オペレーティングシステムで、グラフィカルユーザーインターフェイス（GUI）を提供するためのシステムです。Dockerコンテナ内でGUIを実行するには、ホストのX11ディスプレイにアクセスする必要があります。このため、コンテナがホストのX11サーバーと通信できるように設定を行います。
- **xhostコマンド**: `xhost`コマンドは、X11サーバーのアクセス許可リストを管理するためのツールです。`xhost +local:dockeruser`は、ローカルユーザー（この場合は`dockeruser`）がX11サーバーにアクセスできるようにするコマンドです。

---

## RUN useraddコマンドの解説

### 実行コマンド
```dockerfile
RUN useradd -m -s /bin/bash dockeruser && echo "dockeruser:docker" | chpasswd && adduser dockeruser sudo
```

### 各引数の意味
- `useradd`: 新しいユーザーを作成するコマンド。
  - `-m`: ユーザーのホームディレクトリを作成します。
  - `-s /bin/bash`: ユーザーのシェルを`/bin/bash`に設定します。
- `echo "dockeruser:docker" | chpasswd`: ユーザー`dockeruser`のパスワードを`docker`に設定します。
- `adduser dockeruser sudo`: ユーザー`dockeruser`を`sudo`グループに追加し、管理者権限を付与します。


# `apt` vs `pip` — 役割の使い分けについて

`apt install` と `pip install -r requirements.txt` は、どちらもパッケージをインストールするためのコマンドですが、その役割と目的は根本的に異なります。

## 例えるなら「スーパー」と「専門店のレシピ」

- **`apt` は「街の大きなスーパー」:**
  - OS（オペレーティングシステム）が機能するために必要な、あらゆるものが揃っています。
  - 例えば、`git` や `terminator` のようなツール、プログラムをビルドするためのコンパイラ（開発ツール）など、システム全体の基盤となるソフトウェアです。
  - OSの管理者によってテストされた安定したパッケージが提供されます。

- **`pip` と `requirements.txt` は「専門料理のレシピと材料の取り寄せ」:**
  - `requirements.txt` は、特定のPythonプロジェクト（料理）に必要なライブラリ（専門的な材料）のリストです。
  - `pip` はそのリストを見て、PyPI（Python Package Index）という巨大なオンラインの専門店街から、必要なPythonライブラリを正確に取り寄せます。`numpy` や `pandas` のような、データサイエンスで使う専門的なライブラリも最新版が手に入ります。

---

## メリット・デメリット

### `apt install`

-   **メリット:**
    -   **OSレベルのものを何でもインストール可能:** Pythonライブラリだけでなく、システムツールやコンパイラ、他のプログラムが必要とするシステムライブラリ（例: `libjpeg-dev`）も管理できます。
    -   **安定した依存関係:** OSのバージョンごとにテストされているため、システム全体で見たときの互換性が高く、安定しています。

-   **デメリット:**
    -   **バージョンが古い傾向:** 安定性を最優先するため、パッケージのバージョンが最新でない場合があります。
    -   **Pythonライブラリの品揃えが限定的:** PyPIに比べると、`apt` で提供されているPythonライブラリはごく一部です。

### `pip install -r requirements.txt`

-   **メリット:**
    -   **豊富なPythonライブラリ:** PyPIを通じて、膨大な数のPythonライブラリにアクセスできます。
    -   **自由なバージョン管理:** 最新バージョンを利用したり、プロジェクトごとにバージョンを厳密に固定したりすることが容易です。

-   **デメリット:**
    -   **Pythonライブラリ専用:** OSのツールやシステムライブラリはインストールできません。
    -   **ビルド環境に依存:** ソースコードからコンパイルが必要なライブラリの場合、`apt` などで事前にコンパイラや開発用のヘッダファイルをインストールしておく必要があります。

---

## 結論：どちらがより適しているか

どちらか一方が優れているというわけではなく、**両方を正しく使い分ける**のが最適なアプローチです。

1.  **システム全体の土台作りは `apt`:**
    -   OSの基本ツール (`git`, `sudo`, `curl` など)
    -   ROSの環境 (`ros-humble-desktop`, `ros-dev-tools` など)
    -   `pip` でライブラリをビルドするために必要なもの (コンパイラ、`-dev` が付くパッケージなど)

2.  **個別のPythonプロジェクトが必要とするライブラリは `pip` と `requirements.txt`:**
    -   プログラムコードが直接 `import` して使用するもの (`numpy`, `pandas`, `opencv-python` など)

この2つの役割を理解し、適切に組み合わせることで、クリーンで再現性の高い開発環境を構築できます。

# FlexBE Docker環境 構築・デバッグ備忘録

## 1. コンテナが起動しない・すぐ終了する

**エラー:**
```
exec format error
Exited (0)
```

**原因:**
- `entrypoint.sh` の改行コードがWindows形式(CRLF)だったか、1行目の `#!/bin/bash` が抜けていた。
- コンテナがインタラクティブなシェルに接続されておらず、起動後すぐに正常終了していた。

**対処法:**
- `entrypoint.sh` の改行コードをUnix形式(LF)に修正 (`sed -i 's/\r$//' ...`)。
- `docker-compose.yml` で `tty: true` と `stdin_open: true` を設定する。

## 2. rosdepが初期化されていない・見つからない

**エラー:**
```
rosdep: command not found
ERROR: your rosdep installation has not been initialized yet. Please run: rosdep update
```

**原因:**
- `ros-dev-tools` が未インストールだった。
- `rosdep init` がrootで実行されたため、一般ユーザー(`dockeruser`)から見ると初期化されていない状態だった。

**対処法:**
- Dockerfileで `ros-dev-tools` をインストールする。
- `rosdep init` はDockerfileのroot権限の段階で実行し、ユーザーのスクリプト内では `rosdep update` を実行する。

## 3. FlexBE UIが起動直後にクラッシュする

**エラー:**
```
FATAL:crashpad_linux.cc(...) Check failed: client.StartHandler(...)
```

**原因:**
- Chromium(nwjs)が設定ファイルを書き込もうとしたフォルダ（`~/.config` 等）の権限がなかった。

**対処法:**
- 環境変数 `XDG_CONFIG_HOME` と `XDG_CACHE_HOME` を、書き込み権限のある `/tmp` 以下のディレクトリに変更する。
- この設定は `.bashrc` を読まない場合も考慮して、起動スクリプト自体に `export` 文を記述する。

## 4. UIの表示が乱れ、KMSエラーが発生する

**エラー:**
```
KMS: DRM_IOCTL_MODE_CREATE_DUMB failed: Permission denied
```

**原因:**
- コンテナ内のユーザーが、ホストのGPUデバイス（`/dev/dri/*`）にアクセスする権限を持っていなかった。

**対処法:**
- `docker-compose.yml` で `devices` にGPUデバイスをマウントし、`group_add` で `video` や `render` グループに参加させる。

## 5. Docker ComposeでNVIDIA GPUを指定して起動できない

**エラー:**
```
could not select device driver "nvidia" with capabilities: [[all]]
```

**原因:**
- `docker-compose.yml` の `deploy` セクションの書き方が古いか、環境に合っていなかった。

**対処法:**
- `deploy` セクションごと削除して、トップレベルに `runtime: nvidia` を追加する。これが現在の推奨設定。

## 6. AppArmorによりD-Bus通信が拒否される

**エラー:**
```
An AppArmor policy prevents this sender from sending this message to this recipient
```

**原因:**
- セキュリティ機能のAppArmorが、コンテナからホストへのD-Bus通信をブロックしていた。

**対処法:**
- `docker-compose.yml` で `security_opt: ["apparmor=unconfined"]` を設定し、このコンテナのAppArmorを無効化する。

## 7. colcon buildで権限エラーが発生する

**エラー:**
```
PermissionError: [Errno 13] Permission denied: 'log'
```

**原因:**
- `git clone` 等で、ワークスペース内の一部のファイル/ディレクトリの所有者がrootになっていた。

**対処法:**
- `colcon build` の直前に `sudo chown -R dockeruser:dockeruser ~/ros2_ws` を実行して、ワークスペース全体の所有権を現在のユーザーに戻す。

## 8. terminatorのレイアウトが反映されない

**エラー:**
- 設定したレイアウトで起動せず、単一のターミナルで起動する。

**原因:**
- `~/.config/terminator/config` の記述ミス（例: `maximized = true`）。
- 自動実行するスクリプトが入力待ち（対話式）になっていて、処理が止まっていた。

**対処法:**
- 設定を正しいキー（`window_state = maximise`）に修正する。
- 自動起動するスクリプトは、入力待ちをしない非対話式にする。

## 9. Ctrl+CでFlexBEが綺麗に終了しない

**エラー:**
- UIは消えるが、他のノードが残り、プロンプトが戻ってこない。

**原因:**
- Ctrl+C (SIGINT)を送っても、一部のノードが終了しきれずに残ってしまうため、`ros2 launch` プロセス全体が終了しない。

**対処法:**
- `trap` コマンドでCtrl+Cの挙動を上書きする。
- `ros2 launch` をバックグラウンドで実行し、Ctrl+Cが押されたらプロセスグループ全体にSIGINT、それでもダメならSIGKILLを送る、という二段構えの終了処理を実装する。

# ターミナル表示色 (ANSIエスケープシーケンス)

`echo -e` コマンドなどでターミナルの文字色や背景色を変更する際に使用できる代表的なANSIエスケープシーケンスです。

**基本的な文字色 (前景)**

| 色     | コード (30番台) | 例                                       |
| :----- | :-------------- | :--------------------------------------- |
| 黒     | `\e[30m`        | `echo -e "\e[30m黒い文字\e[0m"`           |
| 赤     | `\e[31m`        | `echo -e "\e[31m赤い文字\e[0m"`           |
| 緑     | `\e[32m`        | `echo -e "\e[32m緑の文字\e[0m"`           |
| 黄     | `\e[33m`        | `echo -e "\e[33m黄色い文字\e[0m"`         |
| 青     | `\e[34m`        | `echo -e "\e[34m青い文字\e[0m"`           |
| マゼンタ | `\e[35m`        | `echo -e "\e[35mマゼンタの文字\e[0m"`     |
| シアン   | `\e[36m`        | `echo -e "\e[36mシアンの文字\e[0m"`       |
| 白     | `\e[37m`        | `echo -e "\e[37m白い文字\e[0m"`           |
| デフォルト | `\e[39m`        | (ターミナルのデフォルト文字色に戻す)       |

**高輝度な文字色 (前景)**

| 色         | コード (90番台) | 例                                           |
| :--------- | :-------------- | :------------------------------------------- |
| 明るい黒 (グレー) | `\e[90m`        | `echo -e "\e[90m明るい黒の文字\e[0m"`       |
| 明るい赤     | `\e[91m`        | `echo -e "\e[91m明るい赤の文字\e[0m"`       |
| 明るい緑     | `\e[92m`        | `echo -e "\e[92m明るい緑の文字\e[0m"`       |
| 明るい黄     | `\e[93m`        | `echo -e "\e[93m明るい黄色の文字\e[0m"`     |
| 明るい青     | `\e[94m`        | `echo -e "\e[94m明るい青の文字\e[0m"`       |
| 明るいマゼンタ | `\e[95m`        | `echo -e "\e[95m明るいマゼンタの文字\e[0m"` |
| 明るいシアン   | `\e[96m`        | `echo -e "\e[96m明るいシアンの文字\e[0m"`   |
| 明るい白     | `\e[97m`        | `echo -e "\e[97m明るい白の文字\e[0m"`       |

**背景色**

| 色     | コード (40番台)  | 例                                           |
| :----- | :--------------- | :------------------------------------------- |
| 黒     | `\e[40m`         | `echo -e "\e[40m黒い背景\e[0m"`               |
| 赤     | `\e[41m`         | `echo -e "\e[41m赤い背景\e[0m"`               |
| 緑     | `\e[42m`         | `echo -e "\e[42m緑の背景\e[0m"`               |
| 黄     | `\e[43m`         | `echo -e "\e[43m黄色い背景\e[0m"`             |
| 青     | `\e[44m`         | `echo -e "\e[44m青い背景\e[0m"`               |
| マゼンタ | `\e[45m`         | `echo -e "\e[45mマゼンタの背景\e[0m"`         |
| シアン   | `\e[46m`         | `echo -e "\e[46mシアンの背景\e[0m"`           |
| 白     | `\e[47m`         | `echo -e "\e[47m白い背景\e[0m"`               |
| デフォルト | `\e[49m`         | (ターミナルのデフォルト背景色に戻す)           |

**高輝度な背景色**

| 色         | コード (100番台) | 例                                               |
| :--------- | :--------------- | :----------------------------------------------- |
| 明るい黒 (グレー) | `\e[100m`        | `echo -e "\e[100m明るい黒の背景\e[0m"`           |
| 明るい赤     | `\e[101m`        | `echo -e "\e[101m明るい赤い背景\e[0m"`           |
| 明るい緑     | `\e[102m`        | `echo -e "\e[102m明るい緑の背景\e[0m"`           |
| 明るい黄     | `\e[103m`        | `echo -e "\e[103m明るい黄色の背景\e[0m"`         |
| 明るい青     | `\e[104m`        | `echo -e "\e[104m明るい青の背景\e[0m"`           |
| 明るいマゼンタ | `\e[105m`        | `echo -e "\e[105m明るいマゼンタの背景\e[0m"`     |
| 明るいシアン   | `\e[106m`        | `echo -e "\e[106m明るいシアンの背景\e[0m"`       |
| 明るい白     | `\e[107m`        | `echo -e "\e[107m明るい白い背景\e[0m"`           |

**リセット**

*   `\e[0m`: 全ての色と装飾（太字、下線など）をリセットします。

**組み合わせ例:**
赤文字で背景を黄色にする場合:
```shellscript
echo -e "\e[31;43m赤文字で背景黄色\e[0m"
```
