# Drake

Model-Based Design and Verification for Robotics.

Please see the [Drake Documentation](https://drake.mit.edu) for more
information.

# git clone 及配置

```
git clone https://github.com/zxw1212/drake.git
  username: zxw1212
  passward: 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  (由github developer setting创建)

确保 git remote -v能看到如下所示:
  origin	https://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@github.com/zxw1212/drake.git (fetch)
  origin	https://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@github.com/zxw1212/drake.git (push)
  upstream	https://github.com/RobotLocomotion/drake.git (fetch)
  upstream	no_push (push)`

如果不是那就运行如下命令行
  git remote remove origin
  git remote add origin https://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@github.com/zxw1212/drake.git
  git remote add upstream https://github.com/RobotLocomotion/drake.git
  git remote set-url --push upstream no_push
```

# 编译

1. 确认依赖
https://drake.mit.edu/from_source.html#mandatory-platform-specific-instructions

其中bazel的安装：

```
sudo apt update
sudo apt install openjdk-11-jdk
sudo apt install pkg-config zip g++ zlib1g-dev unzip python3
wget https://github.com/bazelbuild/bazel/releases/download/6.4.0/bazel-6.4.0-installer-linux-x86_64.sh
chmod +x bazel-4.2.2-installer-linux-x86_64.sh
sudo ./bazel-4.2.2-installer-linux-x86_64.sh
bazel version
```

2. 选择平台

```
sudo ./setup/ubuntu/install_prereqs.sh
```

3. 编译

```
like:
  bazel build //examples/...
```

# 更多参考

源码安装：

```
https://drake.mit.edu/from_source.html
https://drake.mit.edu/bazel.html#developing-drake-using-bazel
```

相关课程：

```
https://manipulation.csail.mit.edu/trajectories.html
https://underactuated.csail.mit.edu/index.html
```


