#include <iostream>
#include <vector>
#include <math.h>
#include <ctime>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>
#include <fstream>
 
using namespace cv;
using namespace std;
 
#define WINDOW_WIDTH 500      //自定义窗口大小的宏 
#define NODENUMBER 120        // 这两个量和主函数里的变量值要保持一致
#define SUBSINKNUMBER 12
#define MAX_SUBSINK 100
#define RANGE 5
 

enum Role {Sensor = 1, Subsink, Sink};   /* 节点的角色定义：感知节点、Subsink节点、Sink节点*/

typedef struct 
{
  
  int y_lower;              /* subsink 节点通信距离的下界  */
  int y_upper;              /* subsink 节点通信距离的下界  */
} subsinkNodeCommu;

typedef struct 
{
  int nodeId;           /* 节点编号，从1开始编号 */
  int x;              /* 节点横坐标  */
  int y;              /* 节点纵坐标  */
  Role role;           /* 节点角色：感知节点、Subsink节点、Sink节点  */
  int subsinkId = 0;       /* 节点所归属的Subsink节点编号，Subsink节点本身归属的Subsink值为0, 默认值  */ 
  int hopCnt;          /* 节点离归属Subsink节点的跳数  */
  int curSubsink;      /* 节点当前选择归属的Subsink节点 */
  int* ToSubsinkList;      /* 保存当前节点到每个subsink节点的最小跳数  */

} sensorNode;

// sensorNode sensorNodes[NODENUMBER+1];   // 节点编号从1开始， 1 - NODENUMBER 


// 计算两点间的距离
float calPointDisance(sensorNode point1, sensorNode point2)
{
  return sqrt((point1.x - point2.x)*(point1.x - point2.x) + (point1.y - point2.y)*(point1.y - point2.y));
}


// 计算Subsink节点的通信距离，目前仅考虑解析式为：x + C = 0的形式, 参数 X 为Subsink节点的 X 坐标值
int calCommuDistance(int CommuRange, int C, int X)
{
 return int(sqrt(CommuRange*CommuRange - (X - abs(C))*(X - abs(C))));
}

/* 判断感知节点区域的连通性 */
bool judgeSensorsConnected(int SubsinkNum, sensorNode sensors[NODENUMBER+1], int CommuRange) 
{
 
 vector<int> floodNodes;
 floodNodes.clear();
 floodNodes.push_back(SubsinkNum + 1);
 int curhopCnt = 0;
 while(!floodNodes.empty())
      {
        curhopCnt++;
        vector<int> newFloodNodes;
        newFloodNodes.clear();
        for(vector<int>::iterator it=floodNodes.begin(); it!= floodNodes.end();  it++)
           {
            for(int i = SubsinkNum + 1; i <= NODENUMBER; i++)
               {
                if(find(floodNodes.begin(), floodNodes.end(), i) != floodNodes.end())
                   continue;
                if(sensors[i].role ==  Sensor)
                  {
                    float temDis = calPointDisance(sensors[i], sensors[*it]);
                    if(temDis <= CommuRange) 
                      {
                        if(sensors[i].hopCnt > curhopCnt )
                          {
                            sensors[i].hopCnt = curhopCnt;
                            newFloodNodes.push_back(i);
                          }
                           
                      }
                  }
               }
           }
        floodNodes.clear();
        floodNodes.assign(newFloodNodes.begin(), newFloodNodes.end());
      }
  for(int i=SubsinkNum+1; i <= NODENUMBER; i++)  
     if (sensors[i].hopCnt == INT_MAX)
        return false;
  return true;  
}

/*
   初始化仿真场景：限定感知节点部署区域为 length*width 的矩形区域（第一象限）， NodeNum 个感知节点随机均匀分布;
   sink节点的移动轨迹为一条直线，其解析式为：Ax + By + C = 0; 
   sink节点的一跳通信距离为 CommuRange，在该通信范围内的节点均为 Subsink 节点;
   向量 subsinkIDSet 为保存的所有 subsink 节点的ID号
   变量SubsinkNum 为预定的Subsink节点的数目(Subsink节点的编号从1到SubsinkNum)，即随机生成个SubsinkNum个subsink节点，而后每个Subsink节点直接关联一个感知节点，进而保证感知节点连通
*/
void CreateScenario(int NodeNum, int Length, int Width, int A, int B, int C, int CommuRange, vector<int>& subsinkIDSet, int SubsinkNum, sensorNode sensorNodes[NODENUMBER+1], subsinkNodeCommu subsinkCommu[])
{
 float delayTime = 0.0;
 srand(time(NULL));
 int leftX = max(abs(C) - CommuRange, 0);
 int rightX = min(abs(C) + CommuRange, Length);
 int deltaY = Width/(SubsinkNum*2);
 cout<<"Increment: "<< deltaY<<endl;
 for(int i=1; i<= SubsinkNum; i++)  // 产生相应数量(SubsinkNum)的Subsink节点
    {
      //sensorNodes[i].x = rand() % (rightX - leftX + 1) + leftX;
      sensorNodes[i].x = rand()% (2*RANGE) - RANGE + (leftX + rightX)/2;  // 随机产生在中心点附近正负RANGE区域内的X坐标
      //sensorNodes[i].y = rand() % (deltaY + 1) + (i-1)*deltaY;
      sensorNodes[i].y = rand() % (RANGE) + (i+1-1)*deltaY*2;
      sensorNodes[i].nodeId = i;
      sensorNodes[i].hopCnt = 0;        // Subsink节点跳数初始为0
      sensorNodes[i].role = Subsink;
      sensorNodes[i].curSubsink = 0;    //Subsink节点所属Subsink节点初始化为0
      subsinkIDSet.push_back(i);

      subsinkCommu[i].y_lower = max(0, sensorNodes[i].y - int(sqrt(CommuRange*CommuRange - (sensorNodes[i].x - abs(C))*(sensorNodes[i].x - abs(C)))));
      subsinkCommu[i].y_upper = min(Width, sensorNodes[i].y + int(sqrt(CommuRange*CommuRange - (sensorNodes[i].x - abs(C))*(sensorNodes[i].x - abs(C)))));
    }
 cout<<"111111"<<endl;
 cout<<"leftX = "<<leftX<<endl;
 cout<<"rightX = "<<rightX<<endl;
 if(leftX == 0)    // Subsink节点分布居左端
    {
      for(int j = SubsinkNum + 1; j <= 2 * SubsinkNum; j++) // 产生与每个Subsink节点直接相邻的一跳感知节点
        {
          sensorNodes[j].x = rand() % (sensorNodes[j - SubsinkNum].x + 1) + CommuRange;
          int tempX = sensorNodes[j - SubsinkNum].x;
          int tempY = sensorNodes[j - SubsinkNum].y;
          int lowY = max(0, (int)(tempY - sqrt((2*CommuRange - tempX)*tempX)));
          int highY = min(Width, (int)(tempY + sqrt((2*CommuRange - tempX)*tempX)));
          int finalX = sensorNodes[j].x;
          int finalY;
          do
            {
             srand(time(NULL));
             finalY = rand() % (highY - lowY + 1) + lowY;

            }while(sqrt((finalX - tempX)*(finalX - tempX) + (finalY - tempY)*(finalY - tempY)) > (float)CommuRange);
          sensorNodes[j].y = finalY;
          sensorNodes[j].hopCnt = INT_MAX;
          sensorNodes[j].nodeId = j;
          sensorNodes[j].role = Sensor;
          sensorNodes[j].curSubsink = INT_MAX;  // 感知节点的所属SUbsink节点初始化为无穷大
          cout<<333333<<endl;
        }
        cout<<"22222"<<endl;
        bool connectedGraph = false;
        do
          {
            for(int k = 2*SubsinkNum + 1; k <= NodeNum; k++)  // 在余下的感知节点空间里生成连通的随机感知节点
               {
                 sensorNodes[k].x = rand() % (Length - CommuRange + 1) + CommuRange;
                 sensorNodes[k].y = rand() % (Width + 1);
                 sensorNodes[k].nodeId = k;
                 sensorNodes[k].hopCnt = INT_MAX;        // 节点跳数初始为无穷大
                 sensorNodes[k].curSubsink = INT_MAX;    // 感知节点的所属SUbsink节点初始化为无穷大
                 sensorNodes[k].role = Sensor;
               }
           sensorNode tmpSensorNodes[NODENUMBER+1];
           for(int i = 0; i<=NODENUMBER; i++)
              tmpSensorNodes[i].ToSubsinkList = new int[SubsinkNum+1];
           memcpy(tmpSensorNodes, sensorNodes, sizeof(sensorNodes)); 
           connectedGraph = judgeSensorsConnected(SubsinkNum, tmpSensorNodes, CommuRange);
          }while(!connectedGraph);
       

    }
  else if (rightX == Length) // Subsink节点分居在右端
         {
           for(int j = SubsinkNum + 1; j <= 2 * SubsinkNum; j++) // 产生与每个Subsink节点直接相邻的一跳感知节点
               {
                sensorNodes[j].x = rand() % (Length - sensorNodes[j - SubsinkNum].x + 1) + sensorNodes[j - SubsinkNum].x - CommuRange;
                int tempX = sensorNodes[j - SubsinkNum].x;
                int tempY = sensorNodes[j - SubsinkNum].y;
                int lowY = max(0, (int)(tempY - sqrt((2*CommuRange + tempX - Length)*(Length - tempX))));
                int highY = min(Width, (int)(tempY + sqrt((2*CommuRange + tempX - Length)*(Length - tempX))));
                int finalX = sensorNodes[j].x;
                int finalY;
               do
                 {
                  srand(time(NULL));
                  finalY = rand() % (highY - lowY + 1) + lowY;

                 }while(sqrt((finalX - tempX)*(finalX - tempX) + (finalY - tempY)*(finalY - tempY)) > (float)CommuRange);
               sensorNodes[j].y = finalY;
               sensorNodes[j].hopCnt = INT_MAX;
               sensorNodes[j].nodeId = j;
               sensorNodes[j].role = Sensor;
               sensorNodes[j].curSubsink = INT_MAX;    // 感知节点的所属SUbsink节点初始化为无穷大
               cout<<444444<<endl;
              }
           bool connectedGraph = false;
           do
             {
              for(int k = 2*SubsinkNum + 1; k <= NodeNum; k++)  // 在余下的感知节点空间里生成连通的随机感知节点
               {
                 sensorNodes[k].x = rand() % (Length - CommuRange + 1);
                 sensorNodes[k].y = rand() % (Width + 1);
                 sensorNodes[k].nodeId = k;
                 sensorNodes[k].hopCnt = INT_MAX;        // 节点跳数初始为无穷大
                 sensorNodes[k].curSubsink = INT_MAX;    // 感知节点的所属SUbsink节点初始化为无穷大
                 sensorNodes[k].role = Sensor;
                 
               }
              sensorNode tmpSensorNodes[NODENUMBER+1];
              for(int i = 0; i<=NODENUMBER; i++)
                 tmpSensorNodes[i].ToSubsinkList = new int[SubsinkNum+1];
              memcpy(tmpSensorNodes, sensorNodes, sizeof(sensorNodes)); 
              connectedGraph = judgeSensorsConnected(SubsinkNum, tmpSensorNodes, CommuRange);
             }while(!connectedGraph);
         }
  else 
    ;
 }




/*
画出节点的图示，包括感知节点，Subsink节点，sink节点移动轨迹，Subsink节点区域
*/
void drawScenario(int nodeNum, int A, int B, int C, int CommuRange, int radius_Sensor, int radius_Subsink, sensorNode sensorNodes[NODENUMBER+1])
{
  Mat img = Mat::zeros(WINDOW_WIDTH, WINDOW_WIDTH, CV_8UC3);
  rectangle(img, Point(5,5), Point(WINDOW_WIDTH-5, WINDOW_WIDTH-5), Scalar(255, 0, 0), 1);     // 画坐标轴
 

  if(A == 0)
    {
      line(img, Point(5,-C/B), Point(WINDOW_WIDTH-5, -C/B), Scalar(0,255,0), 1);  // 画 sink 节点移动轨迹
      line(img, Point(5,-C/B-CommuRange), Point(WINDOW_WIDTH-5, -C/B-CommuRange), Scalar(0,255,0), 1);  // 画Subsink节点区域
      line(img, Point(5,-C/B+CommuRange), Point(WINDOW_WIDTH-5, -C/B+CommuRange), Scalar(0,255,0), 1);
    }
  if(B == 0)
    {
      line(img, Point(-C/A, 5), Point(-C/A, WINDOW_WIDTH-5), Scalar(0,255,0), 1);  // 画 sink 节点移动轨迹
      line(img, Point(-C/A-CommuRange, 5), Point(-C/A-CommuRange, WINDOW_WIDTH-5), Scalar(0,255,0), 1);  // 画Subsink节点区域
      line(img, Point(-C/A+CommuRange, 5), Point(-C/A+CommuRange, WINDOW_WIDTH-5), Scalar(0,255,0), 1);
     }

  for(int i= 1; i <= nodeNum; i++)
     {
       if (sensorNodes[i].role == Sensor)
          circle(img, Point(sensorNodes[i].x, sensorNodes[i].y), radius_Sensor, Scalar(0,0,255), -1);
       else if(sensorNodes[i].role == Subsink)
          circle(img, Point(sensorNodes[i].x, sensorNodes[i].y), radius_Subsink, Scalar(0,0,255), -1);
     }

  imshow("Scenario", img);
	waitKey(0);
}

// 建立感知节点到最佳Subsink节点（存在多个Subsink节点）的信息
void MultiSubsinkflooding(vector<int>& subsinkIDSet, float CommuRange, sensorNode sensorNodes[NODENUMBER+1])
{
 int floodingCnt = subsinkIDSet.size();
 int wait_schemeCnt = subsinkIDSet.size();
 vector<int> floodNodes;
 floodNodes.clear();
 floodNodes.assign(subsinkIDSet.begin(), subsinkIDSet.end());
 int curhopCnt = 0;
 while(!floodNodes.empty())
      {
        curhopCnt++;
        vector<int> newFloodNodes;
        newFloodNodes.clear();
        for(vector<int>::iterator it=floodNodes.begin(); it!= floodNodes.end();  it++)
           {
            for(int i = 1; i <= NODENUMBER; i++)
               {
                if(find(floodNodes.begin(), floodNodes.end(), i) != floodNodes.end())
                   continue;
                if(sensorNodes[i].role ==  Sensor)
                  {
                    float temDis = calPointDisance(sensorNodes[i], sensorNodes[*it]);
                    if(temDis <= CommuRange) 
                      {
                        floodingCnt++;                      // 出现在节点洪泛区域内的节点，都需要接收洪泛包以确认是否需要更新，以此来评价洪泛机制的冗余包传递数量
                        if(sensorNodes[i].hopCnt > curhopCnt )
                          {
                            wait_schemeCnt++;
                            sensorNodes[i].hopCnt = curhopCnt;
                            if(sensorNodes[*it].role == Subsink)  // 上一跳节点为subsink
                               sensorNodes[i].curSubsink = *it;
                            else
                                sensorNodes[i].curSubsink = sensorNodes[*it].curSubsink;
                            newFloodNodes.push_back(i);
                          }
                           
                      }
                  }
               }
           }
        floodNodes.clear();
        floodNodes.assign(newFloodNodes.begin(), newFloodNodes.end());
       }
  cout<<endl<<"洪泛次数："<<floodingCnt<<endl;
  cout<<"数据包传输次数："<<wait_schemeCnt<<endl;

}


// 建立感知节点到每个Subsink节点的跳数值
void SingleSubsinkflooding(int subsinkID, int subsinkNum, float CommuRange, sensorNode sensors[NODENUMBER+1], sensorNode sensorNodes[NODENUMBER+1])
{
  
 vector<int> floodNodes;
 floodNodes.clear();
 floodNodes.push_back(subsinkID);
 int curhopCnt = 0;
 while(!floodNodes.empty())
      {
        curhopCnt++;
        vector<int> newFloodNodes;
        newFloodNodes.clear();
        for(vector<int>::iterator it=floodNodes.begin(); it!= floodNodes.end();  it++)
           {
            for(int i = 1; i <= NODENUMBER; i++)
               {
                if(find(floodNodes.begin(), floodNodes.end(), i) != floodNodes.end())
                   continue;
                if(sensors[i].role ==  Sensor)
                  {
                    float temDis = calPointDisance(sensors[i], sensors[*it]);
                    if(temDis <= CommuRange) 
                      {
                                             
                        if(sensors[i].hopCnt > curhopCnt )
                          {
                            sensors[i].hopCnt = curhopCnt;
                            sensors[i].ToSubsinkList[subsinkID] = curhopCnt;   // 感知节点到对应的subsink节点的跳数
                            newFloodNodes.push_back(i);
                            

                          }
                           
                      }
                  }
               }
           }
        floodNodes.clear();
        floodNodes.assign(newFloodNodes.begin(), newFloodNodes.end());
     }
  for(int j = subsinkNum+1; j <= NODENUMBER; j++)
      sensorNodes[j].ToSubsinkList[subsinkID] = sensors[j].ToSubsinkList[subsinkID];
     
 }


bool JudgeConnected(sensorNode sensors[])
{
  for(int i=1; i <= NODENUMBER; i++)  
     if (sensors[i].hopCnt == INT_MAX)
        return false;
  return true;  
}



// 由保存的节点坐标文件生成对应的节点数据结构
void restoreFromFile(string fileName,int SubsinkNum, sensorNode sensorNodes[NODENUMBER+1])
{
ifstream infile(fileName, ios::in);
if(!infile.fail())
  {
    
    string str;
    getline(infile, str);
    int i = 1;
    while(!infile.eof())
        {
          int num;
          infile >> num;
          sensorNodes[i].nodeId = num;
          cout<< num<<"\t";
          infile >> num;
          sensorNodes[i].x = num;
          cout<< num<<"\t";
          infile >> num;
          sensorNodes[i].y = num;
          cout<< num<<endl;
          if(i <= SubsinkNum)
            { 
              sensorNodes[i].role = Subsink;
              sensorNodes[i].hopCnt = 0;
            }
          else
              {
                sensorNodes[i].role = Sensor;
                sensorNodes[i].hopCnt = INT_MAX;
              }
          sensorNodes[i].curSubsink = 0;
          i++;
        }
  }
}


/* 统计每个Subsink节点的通信距离长度，其思路参考郜帅等人的论文：Sink节点一旦进入其他Subsink节点的通信范围，就马上切断与当前Subsink节点的通信 
   subsinkCommu: 为Subsink节点与Sink节点通信的开始与结束
   SubsinkCommuDis：为本文沿用的方法保存每个Subsink节点的通信时长，即距离
*/
bool CalCommuDistance(subsinkNodeCommu subsinkCommu[], vector<int>& SubsinkCommuDis, int SubsinkNum, int Length)
{
 int i = 1;
 for(; i < SubsinkNum; i++)
   {
     if (subsinkCommu[i+1].y_lower > subsinkCommu[i].y_upper)
        return false;
     else
          SubsinkCommuDis.push_back(subsinkCommu[i+1].y_lower - subsinkCommu[i].y_lower); 
   }
 if(subsinkCommu[i].y_upper >= Length)
     SubsinkCommuDis.push_back(subsinkCommu[i].y_upper - subsinkCommu[i].y_lower);
 else
     return false;
 return true;

}






int main(void)
{

int Nodenumber =  120;      /* 节点数目，参考文献所取值是：120-200之间，隔10递增 */
int Length  = 300;        /* 节点分布区域长度 */
int Width  = 300;          /* 节点分布区域宽度 */

/* Sink节点的移动轨迹，其解析式为：Ax + By + C = 0  */
int  A = 1;
int  B = 0;
int  C = -300;

int CommuRange  = 52;   /* 感知节点通信半径 */
float Speed =  0.8;         /* Sink 节点的移动速度： m/s */
int SubsinkNum = 12;
sensorNode sensorNodes[NODENUMBER+1];
for(int i = 0; i<=NODENUMBER; i++)
    sensorNodes[i].ToSubsinkList = new int[SubsinkNum+1];

int radius_Sensor = 2;
int radius_Subsink = 4;

 vector<int> subsinkIDSet;
 subsinkIDSet.clear();

 vector<int> SubsinkCommuDis;
 SubsinkCommuDis.clear();

 
 subsinkNodeCommu subsinkCommu[SubsinkNum+1];
 CreateScenario(Nodenumber, Length, Width, A, B, C, CommuRange, subsinkIDSet, SubsinkNum, sensorNodes, subsinkCommu);




 // 将生成的感知节点坐标写入文件保存  
 ofstream fileSensors;
 fileSensors.open("sensorInfo.txt", ios::trunc);
 fileSensors << "nodeID"<<"\t"<<"\t"<< "X position" << "\t"<<"Y position";
 for(int i = 1; i <= NODENUMBER; i++)
    {
      fileSensors << endl;
      fileSensors << sensorNodes[i].nodeId<<"\t"<<"\t"<<sensorNodes[i].x << "\t" <<"\t"<< sensorNodes[i].y;
    }
 fileSensors.close();
 //drawScenario(Nodenumber, A, B, C, CommuRange, 2, 4);




  /*
  floodingToSPT(subsinkIDSet, CommuRange);
  vector<int> subsinkIDSetTmp;
  subsinkIDSetTmp.clear();
  vector<int>::iterator it=subsinkIDSet.begin();
  subsinkIDSetTmp.push_back(*it);
  sensorNode sensors[NODENUMBER+1];
  
  memcpy(sensors, sensorNodes, sizeof(sensorNodes));
  SingleSubsinkflooding(subsinkIDSetTmp, CommuRange, sensors);

 if(JudgeConnected(sensors) == true)
    cout<<endl<<"graph is connected!"<<endl;
 else
     cout<<endl<<"graph is not connected!"<<endl;   
 */


/*
 restoreFromFile("sensorInfo.txt", SubsinkNum, sensorNodes);

   sensorNode tmpSensorNodes[NODENUMBER+1];
   memcpy(tmpSensorNodes, sensorNodes, sizeof(sensorNodes));
   
  drawScenario(Nodenumber, A, B, C, CommuRange, radius_Sensor, radius_Subsink, sensorNodes);

 
 for(int i=1; i<=SubsinkNum; i++)
   {
    int subsinkID = i;
    sensorNode tmpSensorNodes[NODENUMBER+1];
    memcpy(tmpSensorNodes, sensorNodes, sizeof(sensorNodes));
    SingleSubsinkflooding(subsinkID, SubsinkNum, CommuRange, tmpSensorNodes, sensorNodes);
    if(JudgeConnected(tmpSensorNodes))
 	      cout<<i<<"The graph is connected!"<<endl;
    else
 	      cout<<i<<"The graph is not connected!"<<endl;
   }
*/


 // 建立感知节点到每个Subsink节点的跳数值
 for(int i=1; i<=SubsinkNum; i++)
   {
    int subsinkID = i;
    sensorNode tmpSensorNodes[NODENUMBER+1];
    memcpy(tmpSensorNodes, sensorNodes, sizeof(sensorNodes));
    SingleSubsinkflooding(subsinkID, SubsinkNum, CommuRange, tmpSensorNodes, sensorNodes);
    if(JudgeConnected(tmpSensorNodes))
 	      {  
          // 若网络拓扑连通，则将感知节点到每个subsink节点的跳数值进行文件保存
          ofstream fileSubsinkHops;
          fileSubsinkHops.open("subsinkHops.txt", ios::trunc);
          for(int i = 1; i <= SubsinkNum; i++)
             for(int j = SubsinkNum + 1; j <= NODENUMBER; j++)
                 fileSubsinkHops << j << "\t" << i << "\t" << tmpSensorNodes[j].ToSubsinkList[i] << endl;
          fileSubsinkHops.close();
          cout<<i<<"The graph is connected!"<<endl;
        }
    else
 	      {
          cout<<i<<"The graph is not connected!"<<endl;
          exit(0);
        }
   }

// 建立感知节点到最佳Subsink节点的信息
MultiSubsinkflooding(subsinkIDSet, CommuRange, sensorNodes);
 cout<<"随机一个节点的信息为："<<endl;
 int k = rand()% NODENUMBER + 1;
 cout<<"随机节点编号："<<k<<endl;
 cout<<"节点的当前所属Subsink: "<<sensorNodes[k].curSubsink<<endl;
 cout<<"节点的当前跳数为："<< sensorNodes[k].hopCnt<<endl;
 for(int i = 1; i<= SubsinkNum; i++)
    cout<<sensorNodes[k].ToSubsinkList[i]<<endl;

 
 // 打印每个subsink节点的通信距离上下限
 cout<<"Subsink节点的上下限坐标为： "<<endl;
 for(int i = 1; i<= SubsinkNum; i++)
   {
    cout<<subsinkCommu[i].y_lower<<", "<<subsinkCommu[i].y_upper<<endl;

   }
  
  ofstream fileSubsinkTransmit;
  fileSubsinkTransmit.open("subsinkTransmit.txt", ios::trunc);
  
  if(CalCommuDistance(subsinkCommu, SubsinkCommuDis, SubsinkNum, Length))
     {
       for(int i=1; i<= SubsinkNum; i++)
         {
          cout<<"第"<<i<<"个Subsink的通信距离为： "<<SubsinkCommuDis[i-1]<<endl;
          fileSubsinkTransmit << "第"<<i<<"个Subsink的通信距离为： "<< "\t"<< SubsinkCommuDis[i-1]<<endl;
         }
     }
  fileSubsinkTransmit.close();

  drawScenario(Nodenumber, A, B, C, CommuRange, radius_Sensor, radius_Subsink, sensorNodes);
 system("pause");
}