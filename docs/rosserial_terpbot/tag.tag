<?xml version='1.0' encoding='UTF-8' standalone='yes' ?>
<tagfile>
  <compound kind="file">
    <name>Readme.md</name>
    <path>/workspaces/plan-percep-ws/src/rosserial_terpbot/</path>
    <filename>Readme_8md.html</filename>
  </compound>
  <compound kind="file">
    <name>CurrVelListener.cpp</name>
    <path>/workspaces/plan-percep-ws/src/rosserial_terpbot/src/nodes/</path>
    <filename>CurrVelListener_8cpp.html</filename>
    <member kind="function" static="yes">
      <type>static float</type>
      <name>filterVelocity</name>
      <anchorfile>CurrVelListener_8cpp.html</anchorfile>
      <anchor>a7f95eba7cc6eda9f5aeacdeefe605f67</anchor>
      <arglist>(const float raw, float &amp;filter)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>publishOdom</name>
      <anchorfile>CurrVelListener_8cpp.html</anchorfile>
      <anchor>af2e5a8584091bc55e304c451d9e13901</anchor>
      <arglist>(geometry_msgs::Pose2D &amp;data, ros::Publisher &amp;curr_odom_publisher)</arglist>
    </member>
    <member kind="function">
      <type>int</type>
      <name>main</name>
      <anchorfile>CurrVelListener_8cpp.html</anchorfile>
      <anchor>a3c04138a5bfe5d72780bb7e82a18e627</anchor>
      <arglist>(int argc, char **argv)</arglist>
    </member>
    <member kind="variable">
      <type>float</type>
      <name>leftTicks</name>
      <anchorfile>CurrVelListener_8cpp.html</anchorfile>
      <anchor>a417b18ab053907cde266bc1ef3c97c3b</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>float</type>
      <name>rightTicks</name>
      <anchorfile>CurrVelListener_8cpp.html</anchorfile>
      <anchor>a3d9738632b61b67f0060857f8b97f854</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" static="yes">
      <type>static const constexpr float</type>
      <name>alpha</name>
      <anchorfile>CurrVelListener_8cpp.html</anchorfile>
      <anchor>a4dabc3cdfa974a53a35b58ac4497d3c5</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>geometry_msgs::Pose2D</type>
      <name>curr_odom</name>
      <anchorfile>CurrVelListener_8cpp.html</anchorfile>
      <anchor>a728501774eddfb0503468a74ec08cecc</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>TargetPublisher.cpp</name>
    <path>/workspaces/plan-percep-ws/src/rosserial_terpbot/src/nodes/</path>
    <filename>TargetPublisher_8cpp.html</filename>
    <class kind="class">Trajectory</class>
    <member kind="define">
      <type>#define</type>
      <name>USE_LOCAL_TRAJECTORY</name>
      <anchorfile>TargetPublisher_8cpp.html</anchorfile>
      <anchor>a7534f8628a2adf0c2d1cf2817cf47e42</anchor>
      <arglist></arglist>
    </member>
    <member kind="function" static="yes">
      <type>static constexpr float</type>
      <name>convertLinearToRPS</name>
      <anchorfile>TargetPublisher_8cpp.html</anchorfile>
      <anchor>a57f05ffacf2851f97f108c13ffaae85b</anchor>
      <arglist>(const float &amp;linearVelocity)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static constexpr float</type>
      <name>convertRPSToLinear</name>
      <anchorfile>TargetPublisher_8cpp.html</anchorfile>
      <anchor>a69e5957211ff4ffa4ed80f7bf2a8cd30</anchor>
      <arglist>(const float &amp;RPS)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static constexpr float</type>
      <name>getXLinearVelocity</name>
      <anchorfile>TargetPublisher_8cpp.html</anchorfile>
      <anchor>abb2d8596a890633a8d33de6a7c526626</anchor>
      <arglist>(const float &amp;t, const float &amp;T)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static constexpr float</type>
      <name>getT</name>
      <anchorfile>TargetPublisher_8cpp.html</anchorfile>
      <anchor>a29b7792f93c86d79870c176349c0d0a6</anchor>
      <arglist>(const float &amp;xInitial, const float &amp;xFinal)</arglist>
    </member>
    <member kind="function">
      <type>int</type>
      <name>publishToUART</name>
      <anchorfile>TargetPublisher_8cpp.html</anchorfile>
      <anchor>a3b0df06aaae3df659fef76b19d813e6e</anchor>
      <arglist>(int &amp;uart0_filestream, uint8_t *outbuf, int data_length)</arglist>
    </member>
    <member kind="function">
      <type>int</type>
      <name>publishToUart</name>
      <anchorfile>TargetPublisher_8cpp.html</anchorfile>
      <anchor>a479aecaedadf078d409ae752aae7a5bd</anchor>
      <arglist>(int &amp;uart0_filestream, uint8_t *outbuf, T &amp;input)</arglist>
    </member>
    <member kind="function">
      <type>int</type>
      <name>sendGains</name>
      <anchorfile>TargetPublisher_8cpp.html</anchorfile>
      <anchor>a368e0cadfe160639a1f0f7a3b22e4bf7</anchor>
      <arglist>(int &amp;uart0_filestream, terpbot::msgs::Gains &amp;leftGain, terpbot::msgs::Gains &amp;rightGain)</arglist>
    </member>
    <member kind="function">
      <type>int</type>
      <name>sendLocalTrajectory</name>
      <anchorfile>TargetPublisher_8cpp.html</anchorfile>
      <anchor>ad011ec3f7565c9327abd840122c80693</anchor>
      <arglist>(const Trajectory &amp;trajectory, int &amp;uart0_filestream)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>enable_controller</name>
      <anchorfile>TargetPublisher_8cpp.html</anchorfile>
      <anchor>ab8b5665cf07eac3ce3d299ec8090e907</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>disable_controller</name>
      <anchorfile>TargetPublisher_8cpp.html</anchorfile>
      <anchor>a999dc66d2a9313e9fbe55150daf2d83c</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>initGains</name>
      <anchorfile>TargetPublisher_8cpp.html</anchorfile>
      <anchor>ae8cbbf1297838da3cc659aff72aec8d8</anchor>
      <arglist>(terpbot::msgs::Gains &amp;leftGain, terpbot::msgs::Gains &amp;rightGain)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setupSerialPort</name>
      <anchorfile>TargetPublisher_8cpp.html</anchorfile>
      <anchor>a6ffaee91d7b0d823a2b3edf5d10a8c7c</anchor>
      <arglist>(const int &amp;uart0_filestream)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>sendPoint</name>
      <anchorfile>TargetPublisher_8cpp.html</anchorfile>
      <anchor>a06dcf744fffc2be61691109a3d6748da</anchor>
      <arglist>(const terpbot::msgs::Target &amp;targ, int &amp;uart0_filestream)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>rot2wheel</name>
      <anchorfile>TargetPublisher_8cpp.html</anchorfile>
      <anchor>a967ee82b42ba01f107cc13926530d08b</anchor>
      <arglist>(float vel, float ang_vel)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>targetCB</name>
      <anchorfile>TargetPublisher_8cpp.html</anchorfile>
      <anchor>aa1fe1e89ec8a678408d81fa1090d8651</anchor>
      <arglist>(const geometry_msgs::Pose2D::ConstPtr &amp;msg)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>sigint_handler</name>
      <anchorfile>TargetPublisher_8cpp.html</anchorfile>
      <anchor>a258e3b580e688a0cf46e4258525aeaf1</anchor>
      <arglist>(int sig)</arglist>
    </member>
    <member kind="function">
      <type>int</type>
      <name>main</name>
      <anchorfile>TargetPublisher_8cpp.html</anchorfile>
      <anchor>a3c04138a5bfe5d72780bb7e82a18e627</anchor>
      <arglist>(int argc, char **argv)</arglist>
    </member>
    <member kind="variable" static="yes">
      <type>static const constexpr float</type>
      <name>CONTROL_FREQ</name>
      <anchorfile>TargetPublisher_8cpp.html</anchorfile>
      <anchor>a223242b810b83dfa45310239c5b2c33d</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" static="yes">
      <type>static const constexpr float</type>
      <name>TICKS_PER_REV</name>
      <anchorfile>TargetPublisher_8cpp.html</anchorfile>
      <anchor>aa82ce6f0d7def7fa07876d957b996007</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" static="yes">
      <type>static const constexpr float</type>
      <name>DIA_WHEEL</name>
      <anchorfile>TargetPublisher_8cpp.html</anchorfile>
      <anchor>a51ca90c8f05a1e746a3befc458926ea8</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" static="yes">
      <type>static const constexpr float</type>
      <name>MAX_RPM</name>
      <anchorfile>TargetPublisher_8cpp.html</anchorfile>
      <anchor>a4a0647cf6b6d4d3c80c1485b3796ee78</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" static="yes">
      <type>static const constexpr float</type>
      <name>DOUBLE_PI</name>
      <anchorfile>TargetPublisher_8cpp.html</anchorfile>
      <anchor>acd4a84848857c5da1d92647c43eb76fb</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" static="yes">
      <type>static const constexpr float</type>
      <name>VMAX</name>
      <anchorfile>TargetPublisher_8cpp.html</anchorfile>
      <anchor>acd7cd9e132a8d0c0b68269de081a83e9</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" static="yes">
      <type>static const Trajectory</type>
      <name>trajectory</name>
      <anchorfile>TargetPublisher_8cpp.html</anchorfile>
      <anchor>a58344b48a668567cdaf1dcb077cb0437</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>int</type>
      <name>uart0_filestream</name>
      <anchorfile>TargetPublisher_8cpp.html</anchorfile>
      <anchor>a011caa8b8018db295b3db7cb04860521</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>outbuf</name>
      <anchorfile>TargetPublisher_8cpp.html</anchorfile>
      <anchor>abf2a642d6dc8d598aa48d8b9af0c6b0e</anchor>
      <arglist>[50]</arglist>
    </member>
    <member kind="variable">
      <type>terpbot::msgs::Target</type>
      <name>target</name>
      <anchorfile>TargetPublisher_8cpp.html</anchorfile>
      <anchor>aff9c3a670568c37393aa33b56eb30949</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" static="yes">
      <type>static const constexpr float</type>
      <name>left_KU</name>
      <anchorfile>TargetPublisher_8cpp.html</anchorfile>
      <anchor>a3f84b50f95764d476e40f099d09db8ad</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" static="yes">
      <type>static const constexpr float</type>
      <name>left_TU</name>
      <anchorfile>TargetPublisher_8cpp.html</anchorfile>
      <anchor>a7370eda00100b5750c57167b7aeba054</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" static="yes">
      <type>static const constexpr float</type>
      <name>left_kd_coeff</name>
      <anchorfile>TargetPublisher_8cpp.html</anchorfile>
      <anchor>a895fc4952adc149843b08ae70ed47086</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" static="yes">
      <type>static const constexpr float</type>
      <name>right_KU</name>
      <anchorfile>TargetPublisher_8cpp.html</anchorfile>
      <anchor>a2bfece145ba507d2da80eb491c380aa6</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" static="yes">
      <type>static const constexpr float</type>
      <name>right_TU</name>
      <anchorfile>TargetPublisher_8cpp.html</anchorfile>
      <anchor>ae6d94b41d024102732f7657881838b20</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" static="yes">
      <type>static const constexpr float</type>
      <name>right_kd_coeff</name>
      <anchorfile>TargetPublisher_8cpp.html</anchorfile>
      <anchor>a977ff9dbe087ca252ddf38aa7438fde6</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" static="yes">
      <type>static const constexpr float</type>
      <name>left_KU</name>
      <anchorfile>TargetPublisher_8cpp.html</anchorfile>
      <anchor>a3f84b50f95764d476e40f099d09db8ad</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" static="yes">
      <type>static const constexpr float</type>
      <name>left_TU</name>
      <anchorfile>TargetPublisher_8cpp.html</anchorfile>
      <anchor>a7370eda00100b5750c57167b7aeba054</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" static="yes">
      <type>static const constexpr float</type>
      <name>left_kd_coeff</name>
      <anchorfile>TargetPublisher_8cpp.html</anchorfile>
      <anchor>a895fc4952adc149843b08ae70ed47086</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" static="yes">
      <type>static const constexpr float</type>
      <name>right_KU</name>
      <anchorfile>TargetPublisher_8cpp.html</anchorfile>
      <anchor>a2bfece145ba507d2da80eb491c380aa6</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" static="yes">
      <type>static const constexpr float</type>
      <name>right_TU</name>
      <anchorfile>TargetPublisher_8cpp.html</anchorfile>
      <anchor>ae6d94b41d024102732f7657881838b20</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" static="yes">
      <type>static const constexpr float</type>
      <name>right_kd_coeff</name>
      <anchorfile>TargetPublisher_8cpp.html</anchorfile>
      <anchor>a977ff9dbe087ca252ddf38aa7438fde6</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>Trajectory</name>
    <filename>classTrajectory.html</filename>
    <member kind="function">
      <type></type>
      <name>Trajectory</name>
      <anchorfile>classTrajectory.html</anchorfile>
      <anchor>aa340ba80f1f4d1aa39f19f069d5d8089</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="variable">
      <type>std::vector&lt; float &gt;</type>
      <name>wayPoints</name>
      <anchorfile>classTrajectory.html</anchorfile>
      <anchor>a3062227204843ff1cfe2f497b5882913</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>int</type>
      <name>noOfWayPoints</name>
      <anchorfile>classTrajectory.html</anchorfile>
      <anchor>a9e6d99598082a4e1cfbfa2f118c607b4</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>float</type>
      <name>totalTime</name>
      <anchorfile>classTrajectory.html</anchorfile>
      <anchor>a288a767897c70ac192fc39b16050c800</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="page">
    <name>index</name>
    <title>Package level readme:</title>
    <filename>index</filename>
  </compound>
</tagfile>
