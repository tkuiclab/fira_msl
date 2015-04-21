= FIRA 6th =


=== Install ===
* sudo apt-get install ros-hydro-desktop-full
* sudo apt-get install ros-hydro-qt-ros


=== Compile ===
~Strategy

*  cp ~/FIRA_ws/src/strategy/strategy_ui/strategy_qlabel.hpp  ~/FIRA_ws/devel/include/

~Vision

* cp ~/FIRA_ws/src/vision/Interface/vision_qlabel.hpp  ~/FIRA_ws/devel/include/


=== Run ===


* Robot
** rosrun system motion R1


* Remote
** rosrun strategy strategy_ui



=== Check SOP ===
*Remote
** Check Network
*** ping fira-r1   (check R1-R3 robot link)
** Login to Robot
*** ssh fira-r1   or    ssh iclab@fira-r1



*Robot
** Check Network
*** ping 192.168.2.34   or ping fpga     (check FPGA link)
*** ping 192.168.16.16  or ping ipcam     (check IPCam link)
*** ping 192.168.0.201  or ping remote-1  (check Remote link)


=== Notice ===

* Add node, not add package
* Don't push anything to master


=== Name Map ===

* kbehouse, Kung-Han Chen, kbehouse@gmail.com
* zxc455233, Tsung-Han Chang, zxc455233@gmail.com
* sugargit323254, Yi-Heng Yang, lm19930620@gmail.com
* blacknarutoav, Yi-Xian Tone, a95824901@yahoo.com.tw
* jianming1481, Jian-Ming Lin, jianming1481@gmail.com
* jacky820156, Chen-Chieh Yao, jacky820156@gmail.com

=== Test Area ===

- [x] @mentions, #refs, [links](), **formatting**, and <del>tags</del> are supported
- [x] list syntax is required (any unordered or ordered list supported)
- [x] this is a complete item
- [ ] this is an incomplete item

:smile:
