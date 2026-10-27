<img width="1920" height="1536" alt="A_Fingertip_Haptics_Device_for_Rendering_Directional_Motion_Cues_Hanming_ZHANG" src="https://github.com/user-attachments/assets/6c4f0ef1-eb97-4ae5-98e1-cb24e736844a" />
本项目开发了一套电机驱动的高精度摆动平台，用于研究用户在连续运动过程中对方向性触觉刺激的感知能力。系统以Teensy 4.0为控制核心，集成直流电机、编码器、电磁铁及力传感器，通过控制摆动速度与随位置变化的电磁铁输出，产生不同方向与波形的触觉刺激。   
  
针对低速启动困难、传动滑移及换向振荡，系统采用编码器反馈的闭环运动控制，引入重力前馈补偿，根据摆动角度计算补偿PWM，并设置角度相关输出下限与全局输出限幅。同时，结合分段控制、位置死区及融合当前采样与历史位置的递推滤波，抑制反馈噪声引起的控制抖动，将稳态定位误差由约9°降至0.9°，实现30°运动范围与60–120°/s可调速度。  
  
用户实验涵盖不同速度、磁场调制波形与运动方向的组合。参与者确认准备就绪后，系统经过短暂的随机延迟施加刺激。参与者感知到刺激时立即按下Enter键，随后通过方向键报告感知结果。系统自动记录反应时间、回答准确率及实验条件，并通过力传感器与Python上位机实现接触力测量和同步数据记录。   
  
实验系统已支持12名参与者、1440次标准化实验，旨在通过统计分析检验用户辨别刺激方向的表现是否显著高于随机水平。后续研究将进一步探索运动速度与磁场强度对方向辨别能力的影响，确定用户能够可靠辨别方向的感知阈值。   
  
In this project, a device has been developed. It is a motorized swinging platform designed to investigate how users perceive directional haptic stimuli during continuous motion. A Teensy microcontroller controls the platform’s oscillation speed, while an electromagnet generates different position-dependent magnetic field patterns during selected left-to-right or right-to-left travels. In the user study, participants experience stimuli under different combinations of speed, waveform, and direction. After indicating readiness, a stimulus appears after a short randomized delay. Participants press Enter as soon as they detect the stimulus, then report its perceived direction using the left, right, or down arrow key. Reaction time, response accuracy, and experimental conditions are automatically recorded. This user study is aimed to prove statistically that users are able to feel the direction of the stimuli. In the near future, a second user study will find out the threshold in speed and magnetic field intensity, of users being able to statistically accurately perceive the direction.
<img width="660" height="591" alt="hm_device_abcd_1" src="https://github.com/user-attachments/assets/aaf6c0d7-e1a9-4b86-9b90-6f82119eb02c" />
