#include <iostream>
#include <vector>
#include <math.h>
#include <ctime>
#include <fstream>
#include <time.h>

using namespace std;

#define SENSORS 900
#define SUBSINK 100

 

// 感知节点到subsink节点的跳数信息 (SENSORS+1)*(SUBSINK+1), 0不计数
//int sensorToSubsinkHop[SENSORS+1][SUBSINK+1];
int **sensorToSubsinkHop;

// subsink节点的通信时长（以通信时间 S 计数）, 0不计数, 长度为 SUBSINK+1
//int subsinkCommu[SUBSINK+1];
int *subsinkCommu;

// subsink节点的通信时长（以容纳的节点数为衡量）, 0不计数, 长度为 SUBSINK+1
//int subsinkChargeSensors[SUBSINK+1];
int *subsinkChargeSensors;
int members[SUBSINK+1] = {0};

// 感知节点选择subsink节点的概率信息 (SENSORS+1)*(SUBSINK+1), 0不计数
double ** Probability;
int * lastSensor2SubsinkArr; // 上一次的感知节点归属的subsink节点对应关系
int * curSensor2SubsinkArr;  // 当前迭代轮感知节点归属subsink节点对应关系
double alpha = 0.1;
double gamma1 = 0.3;
double beta = 0.5;
double rho = 0.5;
double proLimit = 0.9;
double epsilon = 0.3;


// 初始解 (SENSORS+1)*(SUBSINK+1), 0不计数
int **initialSolution;
int totalDelay = 500;  // 所能容忍的时延限制，以节点的个数计，统计是的比正常的传输时延超出的部分
int Delayremainder;  // 指示还有多少个延时的余量


double f_sc, f_sb, ff_sb;   // f_sc：当前轮搜索解中的最好解（不考虑是否可行）；f_sb：全局目标函数的最佳值（不考虑是否可行）；ff_sb：全局可行的最佳解

double penalty_factor = 0.5; // 目标函数松驰的惩罚系数
int depth = 100;

//禁忌参数
int tabuTenure = 15; // 禁忌长度
int ** TabuList;  // 禁忌表 （SENSORS+1）*（SUBSINK+1）的矩阵

double starting_time;
double Time_limit = 1.0*60;


// 为变量申请空间
void AllocMemory()
{
 sensorToSubsinkHop = new int*[SENSORS+1]; 
 initialSolution = new int*[SENSORS+1]; 
 TabuList = new int*[SENSORS+1]; 
 Probability = new double*[SENSORS+1]; 
 for(int i = 0; i <= SENSORS; i++)
    {
      sensorToSubsinkHop[i] = new int[SUBSINK+1];
      initialSolution[i] = new int[SUBSINK+1];
      TabuList[i] = new int[SUBSINK+1];
      Probability[i] = new double[SUBSINK+1];
    }
 
 subsinkCommu = new int[SUBSINK+1];
 subsinkChargeSensors = new int[SUBSINK+1];
 lastSensor2SubsinkArr = new int[SENSORS+1];
 curSensor2SubsinkArr = new int[SENSORS+1];
}

// 初始化每个感知节点选择Subsink节点的概率为平均值
void InitialProbability()
{
  for(int i = 1; i <= SENSORS; i++)
    for(int j = 1; j <= SUBSINK; j++)
        Probability[i][j] = 1.0/SUBSINK;
}

// 释放变量所占用的空间
void FreeMemory()
{
 delete []subsinkCommu;
 subsinkCommu = NULL;
 
 delete []subsinkChargeSensors;
 subsinkChargeSensors = NULL;

 delete []lastSensor2SubsinkArr;
 lastSensor2SubsinkArr = NULL;

 delete []curSensor2SubsinkArr;
 curSensor2SubsinkArr = NULL;

 for(int i = 0; i <= SENSORS; i++)
    {
      delete []sensorToSubsinkHop[i];   sensorToSubsinkHop[i] = NULL;
      delete []Probability[i];          Probability[i] = NULL;
      delete []initialSolution[i];      initialSolution[i] = NULL;
      delete []TabuList[i];             TabuList[i] = NULL;
    }

}


void readfileHops(string inFile)
{
 ifstream FIC;
 FIC.open(inFile);  // 读取感知节点跳数文件
 if(FIC.fail())
   {
     cout<<"### ERROR Open, File_Name "<<inFile<<endl;
     getchar();
     exit(0);
   }
  if(FIC.eof())
    {
      cout<<"### ERROR open, File_Name "<<inFile<<endl;
      getchar();
      exit(0);
    }
  for(int i = 1; i < SUBSINK+1; i++)
      for(int j = 1; j < SENSORS + 1; j++)
          {
            FIC >> sensorToSubsinkHop[j][i];
            FIC >> sensorToSubsinkHop[j][i];
            FIC >> sensorToSubsinkHop[j][i];
          }
  FIC.close();
}


void readfileCommu(string inFile)
{
 ifstream FIC;
 FIC.open(inFile);  // 读取感知节点跳数文件
 if(FIC.fail())
   {
     cout<<"### ERROR Open, File_Name "<<inFile<<endl;
     getchar();
     exit(0);
   }
  if(FIC.eof())
    {
      cout<<"### ERROR open, File_Name "<<inFile<<endl;
      getchar();
      exit(0);
    }
  for(int i = 1; i < SUBSINK+1; i++)
     {
       string tempStr;
       FIC >> tempStr;
       FIC >> subsinkCommu[i];
     }
  FIC.close();
}


//随机生成一个 0-(n-1) 的整数
int random_int(int n)
{
	return rand() % n;
}


// 显示数组中的元素
//void disPlay(int row, int col, int matrix[SENSORS+1][SUBSINK+1])
void disPlay(int row, int col, int **matrix)
{
 for(int i = 1; i < row; i++)
    {
      cout<<"node "<<i<<"---";
      for(int j = 1; j < col; j++)
        cout<< matrix[i][j]<<" ";
      cout<<endl;
    }
}

// 随机构建初始解
void randomInitialSolution()
{
int i,j;
srand((unsigned)time(NULL));
for(i = 1; i <= SENSORS; i++)
   {
     int randS = 1 + rand() % SUBSINK;
     for(j = 1; j <= SUBSINK; j++)
        {
          if(j == randS)
            initialSolution[i][j] = 1;
          else
              initialSolution[i][j] = 0;
        }

   }
}


// 计算解的目标函数值及整体时延
void compute_FitnessFun(int **solution, int *Obj, int *Delay)
{
 int i,j;
 int tempObj = 0;
 int *tempDelay = new int[SUBSINK+1]();   // 初始化为0

 int tempTotalDelay = 0;
 for(i = 1; i <= SENSORS; i++)
    for(j = 1; j <= SUBSINK; j++)
      {
       if(solution[i][j] == 1)
          {
            tempObj += sensorToSubsinkHop[i][j];
            tempDelay[j] ++;
          }
      }
 for(j = 1; j <= SUBSINK; j++)
    tempTotalDelay += (tempDelay[j] > subsinkChargeSensors[j] ? tempDelay[j]- subsinkChargeSensors[j] : 0);
 *Obj = tempObj;
 *Delay = tempTotalDelay;
 
 delete [] tempDelay;
 tempDelay = NULL;
}


// 计算将感知节点 x 由 subsink 节点 p 重新指泒到 subsink 节点 q 所引起的目标函数值的增量，并同步更新当前的目标函数值以及 Delayremnainder 变量
void reassignNeighborRefresh(int x, int p, int q)
{
 int delta_1, delta_2, delta_3, delta_remainder, deltaDelay;  
 double delta;
 delta_1 = sensorToSubsinkHop[x][q] - sensorToSubsinkHop[x][p];
 delta_2 = members[p] > subsinkChargeSensors[p] ? -1 : 0; 
 delta_3 = members[q] >= subsinkChargeSensors[q] ? 1 : 0;
                
 if(Delayremainder > 0)
      deltaDelay = delta_2 + delta_3;
 else if(Delayremainder < 0)
         deltaDelay = 0;
 else
     {
      if(delta_2 + delta_3 == 1)
        deltaDelay = 1;
      else
           deltaDelay = 0;
     }
  delta = delta_1 + penalty_factor*deltaDelay;
  delta_remainder = delta_2 + delta_3;
  f_sc += delta;
  Delayremainder += delta_remainder;
  
}


/* 扰动: 首先根据更新解的情况去更新概率矩阵，然后再由概率矩阵进行扰动 */
void pertubation()
{
 int i, j;
 int temp1, temp2;
 int smooth_flag = 0;
 int smooth_idx;
 int max_idx;
 int epsilon1000;
 double maxxxx;

 for(i = 1; i <= SENSORS; i++)
    {
      smooth_flag = 0;
      temp1 = lastSensor2SubsinkArr[i];
      temp2 = curSensor2SubsinkArr[i];
      if(temp1 == temp2)
        {
          for(j = 1; j <= SUBSINK; j++)
             {
               if(j == temp1)
                 {
                   Probability[i][j] = alpha + (1.0 - alpha) * Probability[i][j];
                   if(Probability[i][j] > proLimit)
                      {
                        smooth_flag = 1;
                        smooth_idx = j;
                      }
                 }    
                else
                    Probability[i][j] = (1.0 - alpha) * Probability[i][j];
              }
        }
      else
          {
            for(j = 1; j <= SUBSINK; j++)
               {
                 if(j == temp1)
                   Probability[i][j] = (1.0 - gamma1) * (1.0 - beta) * Probability[i][j];  
                 else if(j == temp2)
                         Probability[i][j] = gamma1 + (1.0 - gamma1)* beta / (SUBSINK - 1.0) + (1.0 - gamma1) * (1.0 - beta) * Probability[i][j];
                 else
                      Probability[i][j] = (1.0 - gamma1) * beta / (SUBSINK - 1.0) + (1.0 - gamma1) * (1.0 - beta) * Probability[i][j];
               }
          }
      if(smooth_flag)
         for(j = 1; j <= SUBSINK; j++)
            Probability[i][j] = 1.0 * Probability[i][j] / (1.0 - (1.0 - rho) * Probability[i][smooth_idx]);     
    }
  epsilon1000 = epsilon * 1000;
  for(i = 1; i <= SENSORS; i++)
     {
       maxxxx = -1.0;
       if(random_int(1000) < epsilon1000)   // 随机分配归属subsink节点
         { 
           temp2 = 1 + random_int(SUBSINK);
           if(temp2 == curSensor2SubsinkArr[i])
              continue;
           else
               {
                 reassignNeighborRefresh(i, curSensor2SubsinkArr[i], temp2);
                 initialSolution[i][temp2] = 1;
                 initialSolution[i][curSensor2SubsinkArr[i]] = 0;
                 members[curSensor2SubsinkArr[i]]--;
                 members[temp2]++;
                 curSensor2SubsinkArr[i] = temp2;
               }
         }
       else                  //选择高概率进行分配 
           {
             for(j = 1; j <= SUBSINK; j++)
                {
                  if (Probability[i][j] > maxxxx)
                     {
                       maxxxx = Probability[i][j];
                       max_idx = j;
                     }
                }
             if(max_idx == curSensor2SubsinkArr[i])
                continue;
             else
                 {
                   reassignNeighborRefresh(i, curSensor2SubsinkArr[i], max_idx);
                   initialSolution[i][max_idx] = 1;
                   initialSolution[i][curSensor2SubsinkArr[i]] = 0;
                   members[curSensor2SubsinkArr[i]]--;
                   members[max_idx]++;
                   curSensor2SubsinkArr[i] = max_idx;
                 } 
           }
     }
  for(int i = 1; i <= SENSORS; i++)
     lastSensor2SubsinkArr[i] = curSensor2SubsinkArr[i];
}


// 基于 Reassign 邻域的禁忌搜索
void TabuSearch()
{
 int num_tabu_best, num_best; // 分别记录禁忌域和非禁忌域的最好解个数
 int num;

 // 考虑将感知节点 x 之前的所属 subsink 节点 p 切换到 subsink 节点 q
 int best_x[50], best_p[50], best_q[50];  // 记录最佳解
 int tabu_best_x[50], tabu_best_p[50], tabu_best_q[50], tabu_best_remainder[50], best_remainder[50]; // 记录禁忌域最佳解
 int x, p, q;
 int iter, select;
 double tabu_best_delta, best_delta,  delta;
 int delta_1, delta_2, delta_3, delta_remainder, deltaDelay;     
 int curObj = 0, curDelay = 0, totalTransDelay = 0;
 //int remainder; // remainder 指示还有多少个延时的余量

 int noImprove = 0;
 

 int tongji = 0;

 for(int i = 1; i <= SENSORS; i++)
    for(int j = 1; j <= SUBSINK; j++)
        if(initialSolution[i][j] == 1)
            {
              members[j]++;
              lastSensor2SubsinkArr[i] = j;
              curSensor2SubsinkArr[i] = j; 
            }

 compute_FitnessFun(initialSolution, &curObj, &totalTransDelay);
 curDelay = totalTransDelay > totalDelay ? totalTransDelay - totalDelay : 0;
 Delayremainder = totalTransDelay - totalDelay;

 f_sc = curObj + penalty_factor*curDelay;
 f_sb = f_sc;
 ff_sb = 9999999.0;

 cout<<"Start to computing....."<<endl;
 starting_time = clock();
 iter = 0;
 while(1.0*(clock() - starting_time)/CLOCKS_PER_SEC < Time_limit)
   {
     num = 0;
     tabu_best_delta = 9999999.0;
     best_delta = 9999999.0;
     num_tabu_best = 0;
     num_best = 0;
    
     for(x = 1; x <= SENSORS; x++)
        {
         for(q = 1; q <= SUBSINK; q++)
             if(initialSolution[x][q] == 1)
                p = q;                       // 找出感知节点 x 原先所属的 subsink 节点 p
         for(q = 1; q <= SUBSINK; q++)
            if(!(q == p))
              {
                num ++;
                delta_1 = sensorToSubsinkHop[x][q] - sensorToSubsinkHop[x][p];
                delta_2 = members[p] > subsinkChargeSensors[p] ? -1 : 0; 
                delta_3 = members[q] >= subsinkChargeSensors[q] ? 1 : 0;
                
                if(Delayremainder > 0)
                   deltaDelay = delta_2 + delta_3;
                else if(Delayremainder < 0)
                        deltaDelay = 0;
                else
                    {
                      if(delta_2 + delta_3 == 1)
                        deltaDelay = 1;
                      else
                          deltaDelay = 0;
                    }
                delta = delta_1 + penalty_factor*deltaDelay;
                delta_remainder = delta_2 + delta_3;

                if(TabuList[x][q] <= iter)  // 感知节点 x 从 subsink 节点 p 移动到 subsink 节点 q 没有被禁忌
                  {
                    if(delta < best_delta)
                      {
                        best_x[0] = x;
                        best_p[0] = p;
                        best_q[0] = q;
                        best_remainder[0] = delta_remainder;
                        best_delta = delta;   // 应该是统计每一轮迭代中的最佳值，而不是针对全局所有解的最佳值，所以这时用全局的最佳解在此不合适，还应该是每一轮中的解进行对比
                        num_best = 1;
                      }
                      else if(delta == best_delta && num_best < 50)  // 记录最好的50个值
                             {
                              best_x[num_best] = x;
                              best_p[num_best] = p;
                              best_q[num_best] = q;
                              best_remainder[num_best] = delta_remainder;
                              num_best++;
                             }

                  }
                else if(TabuList[x][q] > iter) // 感知节点 x 从 subsink 节点 p 移动到 subsink 节点 q 被禁忌
                      {
                        if(delta < tabu_best_delta)
                          {
                            tabu_best_x[0] = x;
                            tabu_best_p[0] = p;
                            tabu_best_q[0] = q;
                            tabu_best_remainder[0] = delta_remainder;
                            tabu_best_delta = delta;
                            num_tabu_best  = 1;
                          }
                        else if(delta == tabu_best_delta && num_tabu_best < 50)
                               {
                                 tabu_best_x[num_tabu_best] = x;
                                 tabu_best_p[num_tabu_best] = p;
                                 tabu_best_q[num_tabu_best] = q;
                                 tabu_best_remainder[num_tabu_best] = delta_remainder;
                                 num_tabu_best++;
                               }

                      }
                }
          }
       /* 选取最佳禁忌解如果特赦条件发生，或者是此轮全部是禁忌解 */   
      if(num_tabu_best > 0 && tabu_best_delta < best_delta && (f_sc + tabu_best_delta < f_sb) || num_best == 0)
        {
          f_sc += tabu_best_delta;
          select = rand() % num_tabu_best;
          initialSolution[tabu_best_x[select]][tabu_best_p[select]] = 0;
          initialSolution[tabu_best_x[select]][tabu_best_q[select]] = 1;
          members[tabu_best_p[select]] --;
          members[tabu_best_q[select]] ++;
          Delayremainder += tabu_best_remainder[select];
          curSensor2SubsinkArr[tabu_best_x[select]] = tabu_best_q[select];
          TabuList[tabu_best_x[select]][tabu_best_p[select]] = tabuTenure;
          TabuList[tabu_best_x[select]][tabu_best_p[select]] += iter;
        }
      else   /* 选取本轮最佳的未被禁忌的解 */
          {
            f_sc += best_delta;
            select = rand() % num_best;
            initialSolution[best_x[select]][best_p[select]] = 0;
            initialSolution[best_x[select]][best_q[select]] = 1;
            members[best_p[select]] --;
            members[best_q[select]] ++;
            Delayremainder += best_remainder[select];
            curSensor2SubsinkArr[best_x[select]] = best_q[select];
            TabuList[best_x[select]][best_p[select]] = tabuTenure;
            TabuList[best_x[select]][best_p[select]] += iter;
          }
      iter++;

	 // printf("%5.6lf\n", f_sc); 
      
	  if(Delayremainder <= 0 && f_sc < ff_sb )
       {
         ff_sb = f_sc;
         noImprove = 0;
       
       }
    if (f_sc < f_sb)
      {
        f_sb = f_sc;
        noImprove = 0;
      }
    else
        {
          noImprove ++;
          if(noImprove > depth)
            {
              noImprove = 0;
              pertubation();
              //compute_FitnessFun(initialSolution, &curObj, &totalTransDelay);
             // f_sc = curObj + penalty_factor*(totalTransDelay > totalDelay? totalTransDelay - totalDelay: 0);
             // Delayremainder = totalTransDelay - totalDelay;
              tongji ++;
            }
        }

   }

compute_FitnessFun(initialSolution, &curObj, &totalTransDelay);
cout<<"final objection is: "<< curObj<<endl;
cout<<"Total trans delay is: "<< totalTransDelay<<endl;
cout<<"Delayremainder is: "<< Delayremainder<<endl;
cout<<"best feasible value is: "<<ff_sb<<endl;
cout<<"best infeasible and feasible value is: "<<f_sb<<endl;
cout<<"tongji: "<<tongji<<endl;
}



/************************************************************************************************* 
将划分到subsink节点的通信时长换算成对应所能完成感知节点数据收集的个数
0.2kbps * T(采集周期) * （subsink归属节点 + 1）     20kbps * T(subsink节点传输时间)
若取T(采集周期) = 300S，则换算成
subsink归属节点 =  （20kbps * T(subsink节点传输时间)）/(0.2kbps * T(采集周期)) -1
               = T(subsink节点传输时间)/3 -1

在此基础上，需要考虑subsink节点在通信时长内能否完成全部节点的数据传输问题，根据所要解决的问题进行设定
即若按T(采集周期) = 200S 计算，所有subsink节点的通信时长内，是可以完成全部感知节点的数据收集的；
而若按T(采集周期) = 300S 计算，所有subsink节点的通信时长内，无法支撑全部感知节点的数据收集。
**************************************************************************************************/

int main(void)
{

AllocMemory();
readfileHops("subsinkHops.txt");
disPlay(SENSORS+1, SUBSINK+1, sensorToSubsinkHop);

readfileCommu("subsinkTransmit.txt");


for(int i=1; i<= SUBSINK; i++)
   subsinkChargeSensors[i] = floor(subsinkCommu[i]/3) - 1;  // T(采集周期) = 300S

for(int i=1; i<=SUBSINK; i++)
  cout<<subsinkChargeSensors[i]<<"  ";

InitialProbability();
randomInitialSolution();
disPlay(SENSORS+1, SUBSINK+1, initialSolution);

int obj, delay;
compute_FitnessFun(initialSolution, &obj, &delay);
cout<<"objective function is : "<<obj<<endl;
cout<<"delay is :"<<delay<<endl;

TabuSearch();

printf("%5.6lf\n", f_sb);
FreeMemory();
system("pause");
return 0;

}