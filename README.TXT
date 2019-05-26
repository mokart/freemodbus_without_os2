串口1是调试串口PA9TX PA10RX  
串口2是收发串口PA2TX PA3RX
发送读设备地址命令
02 03 00 02 00 01 25 F9 


?add
commit
push

example git commit -m "understand how stage works"


要关联一个远程库，使用命令git remote add origin git@server-name:path/repo-name.git；
关联后，使用命令git push -u origin master第一次推送master分支的所有内容；
此后，每次本地提交后，只要有必要，就可以使用命令git push origin master推送最新修改；

  git remote add origin git@github.com:mokart/modbus_without_os.git

第一次 git push -u origin master
以后   git push origin master


如果遇到SL安全，则
git config --global http.sslVerify false


