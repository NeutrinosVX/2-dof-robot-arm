#include <Servo.h>
Servo myservo1,myservo2,myservo3,myservo4;
int pos1 = 0;  //每个舵机的角度值，12345分别对应：底座，大臂，小臂，尖端，爪子舵机
int pos2 = 0;
int pos3 = 130; 
int pos4 = 80;
int speed = 7;  //机械臂运动速度
//setup里记得写连接舵机初始引脚，还有write初始角度pos变量
//2 电位器
int potpin1=A0; //potentiometer pin:电位器引脚
int potpin2=A1; //这里开个变量存引脚，是为了代码容易阅读
int potpin3=A2; //若直接写入具体引脚，引脚多了后，鬼知道这些引脚是干嘛用的
int potpin4=A3;
int val1,val2,val3,val4;   //存取电位器模拟引脚的数值
//使用电位器，setup中不需要写额外内容
//3 记录路径信息结构体
struct memory{   //记录操纵电位器的过程中，每个舵机运动的初始与结束角度
  int num;       //舵机编号
  int st;        //初始变化角度；
  int ed;        //最终变化角度
}a[200];
int k=1;  //记录结构体储存记录数量，后面复位时要使用

int con=0;                          //control：表示现在是哪个舵机在被控制,赋值为01234，0代表初始值
int fct1=-1,fct2=-1,fct3=-1,fct4=-1;//flag continue：表示舵机持续运动状态
                                    //-1:关闭，学习模式不可用；0:待机；非0：运动
/*
简称穿梭法：记录每个舵机每一次运动开始与运动结束时的角度位置，复位时起点直达重点。
优点：复位速度流畅，支持记录复杂操作，支持同时控制多个电位（容易撑爆数组），可输出舵机运动角度表
缺点：因为本方法的性质，会自动过滤一些无效操作，无法完全复制机械臂行动，比如同一个舵机运动起点与终点一致的运动
*/


/*
读取舵机角度参数，控制舵机流畅的回到初始位置
*/
void init_pos(){
  pos1=myservo1.read();  //当前各舵机角度值
  pos2=myservo2.read();
  pos3=myservo3.read();
  pos4=myservo4.read();

  for(int i=pos1;i>=0;i--){  //1号舵机，归位0
    myservo1.write(i);
    delay(speed);
  }myservo1.write(0);

  for(int i=pos3;i<=130;i++){    //3号舵机，归位130
    myservo3.write(i);
    delay(speed);
  }myservo3.write(130);

  for(int i=pos2;i>=0;i--){   //2号舵机，归位0
    myservo2.write(i);
    delay(speed);
  }myservo2.write(0);
  
  for(int i=pos4;i<=80;i++){  //4号舵机，归位80°
    myservo4.write(i);
    delay(speed);
  }myservo4.write(80);

  pos1=0;//更新pos
  pos2=0;
  pos3=130;
  pos4=80;

}

/*
一个固定路径的机械臂抓取函数，可设置旋转度数spin。
我把这个设置成了开机动画，开机一动不动像木头一样太无趣了
*/
void catchs(int spin) {
  
 //先下放小臂
 for (pos3 = 130; pos3 >= 30; pos3 -= 1) {
    myservo3.write(pos3);
    delay(speed);
 }
  //打开爪子
 for (pos4 = 80; pos4 >= 0; pos4 -= 1) {
    myservo4.write(pos4);
    delay(speed);
 }
 //放大臂
 for (pos2 = 0; pos2 <= 70; pos2 += 1) {
    myservo2.write(pos2);
    delay(speed);
 }
 //夹取物品
 for (pos4 = 0; pos4 <= 80; pos4 += 1) {
    myservo4.write(pos4);
    delay(speed);
 }
   //抬起大臂
 for (pos2 = 70; pos2 >= 0; pos2 -= 1) {
    myservo2.write(pos2);
    delay(speed);
 }
  //转向
  for(int i=0;i<spin;i++){   //转动spin度，spin不同颜色可以手动设置转动区间
    myservo1.write(pos1++);
    delay(speed);
  } 
 //大臂放下
 for (pos2 = 0; pos2 <= 15; pos2 += 1) {
    myservo2.write(pos2);
    delay(speed);
 }
 //打开爪子
 for (pos4 = 80; pos4 >= 0; pos4 -= 1) {
    myservo4.write(pos4);
    delay(speed);
 }
  init_pos();//机械臂流畅复位
}

/*
重复判定是否操纵了电位器
*/
void loop(){
  //电位器底座控制
  val1=analogRead(potpin1);
  val1=map(val1,0,1023,0,120);

  //if中表示当检测到1号舵机有操作时的情况：
  if(abs(myservo1.read()-val1)>=5){  //收舵机精度与map映射误差影响，防止连续多次记录1°记录,abs是求绝对值。
       con=1;                   //现在控制的是1号舵机；
       if(!fct1){              //记录1号舵机初始状态，fct1==0时才开始。
         fct1=k;               //在本次状态结束时，k可能已发生变化，不能a[k].ed记录，这里顺手废物利用一下fct1充当下标
         a[k].num=1;
         a[k].st=myservo1.read();//记录最早的一方
         k++;                  //k自在记录到一次开始后就增加。
       }
       myservo1.write(val1);  //先记录，再写入；否则a[k].st记录不到read()的角度了
  }else{                       
    //else中表示当1号舵机无操作时：
    //(1)如果234舵机被控制了,并且1号舵机操纵态为打开，应结束1号机的运动并记录：
    //   if条件:   con！=0,1说明现在其他舵机被控，fct1>0则说明1号舵机与运动态为打开；
    //(2)如果1号机仍然被控，但是触控板按下结束，也需结束1号机的运动并记录：
    //   因为穿梭法记录原理：当下一个舵机被控时，才会结束上次的记录，所以最后一次是记不上的，
    //   需要特判学习模式结束时记录最后一次的结束操作；
    //   if条件:   con=1和触控板结束按下，可以确定本次为最后一次操作
    if((con!=1&&con!=0&&fct1>0)||((con==1)&&(ed_touchval==HIGH))){
      a[fct1].ed=val1;                             
      cout_road(fct1);    //打印本次路径的方法，函数在下方
      fct1=0;             //1号机运动态，归0。待机等待下一次记录；
    }
  }
  
  //电位器大臂控制
  val2=analogRead(potpin2);
  val2=map(val2,0,1023,0,70);
  //注意，因为穿梭法依据下一个舵机被控来结束上一次记录，并且fct1234都设为全局变量，
  //所以234号舵机不能写一个通用的函数方法调用，必须一个个写

  if(abs(myservo2.read()-val2)>=5){
       con=2;
       if(!fct2){
         fct2=k;
         a[k].num=2;
         a[k].st=myservo2.read();
         k++;
       }
       myservo2.write(val2);
  }else{
    if((con!=2&&con!=0&&fct2>0)||((con==2)&&(ed_touchval==HIGH))){
         a[fct2].ed=val2;
         cout_road(fct2);//测试！
         fct2=0;
    }
  }

  //电位器小臂控制
  val3=analogRead(potpin3);
  val3=map(val3,0,1023,30,130);

  if(abs(myservo3.read()-val3)>=10){
       con=3;
       if(!fct3){
         fct3=k;
         a[k].num=3;
         a[k].st=myservo3.read();
         k++;
       }
       myservo3.write(val3);
  }else{
    if((con!=3&&con!=0&&fct3>0)||((con==3)&&(ed_touchval==HIGH))){
         a[fct3].ed=val3;
         cout_road(fct3);//测试！ 
         fct3=0;
    }
  }

  //电位器爪子控制
  val4=analogRead(potpin4);
  val4=map(val4,0,1023,0,80);
  
  if(abs(myservo4.read()-val4)>=8){
       con=4;
       if(!fct4){
         fct4=k;
         a[k].num=4;
         a[k].st=myservo4.read();
         k++;
       }
       myservo4.write(val4);
  }else{
    if((con!=4&&con!=0&&fct4>0)||((con==4)&&(ed_touchval==HIGH))){
         a[fct4].ed=val4;
         cout_road(fct4);//测试！ 
         fct4=0;
    }
  }
  //电位器底座控制与信息记录
  //测试每个舵机实时角度，可观察电位器操作引起的舵机角度变化
  // delay(15);   
  // Serial.print("val1:\t");
  // Serial.print(val1);
  // Serial.print("\tval2:\t");
  // Serial.print(val2);
  // Serial.print("\tval3\t");
  // Serial.print(val3);
  // Serial.print("\tval4:\t");
  // Serial.println(val4);
  
}
void idenservo(int n,int m){
  if(n==1) myservo1.write(m); 
  if(n==2) myservo2.write(m); 
  if(n==3) myservo3.write(m); 
  if(n==4) myservo4.write(m); 
}
/*
回退函数，把结构体中记录的角度值回退改变；
读取结构体中记录的k次次记录并写入
*/
void back(){
    //注意从打到小，还是从小到大
    for(int i=1;i<k;i++){
      int r=a[i].num;
      if(a[i].st<=a[i].ed){
        for(int j=a[i].st;j<=a[i].ed;j++){
          idenservo(r,j);//r号舵机写入角度j
          delay(speed);
        }
      }else{
        for(int j=a[i].st;j>=a[i].ed;j--){
          idenservo(r,j);
          delay(speed);
        }
      } 
    }
}


