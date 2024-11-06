#include "DataManager.h"
/*
 * @Author: younger
 * @Date: 2023-04-26 20:02:05
 * @Last Modified by: younger
 * @Last Modified time: 2023-04-27 00:04:53
 */
#ifndef _BASIC_INFORMATION_H_

#define _BASIC_INFORMATION_H_

/***
 *
 * 声明全局信息
 *
 * 声明函数
 * 1、生成数据(在类中)
 * 2、比较函数(不在类中)
 * 3、距离函数（不在类中）
 */

#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>
#include <map>
#include <algorithm>
#include <regex>
#include <string>
#include <cmath>
#include <math.h>
#include <random>
#include <time.h>

using namespace std;
double avg_task_sati = 0;
double avg_work_sati = 0;
double avg_work_match_num = 0;
double avg_task_match_num = 0;
double avg_runtime = 0;
int sum_repeat = 0;

#define PI 3.1415926535897932384626433832795 #define EARTH_RADIUS 6378.137
int global_Current_workID = 0;
int global_Current_taskNumber = 0;

struct POI
{
    double X;
    double Y;
};

struct WORKER
{
    vector<POI> trajectory;
    double startTime;
    double endTime;
    double range;
    double score;
    double ADdis;
};

struct TASK
{
    double X;
    double Y;
    double Reward;
    double Minscore;
    double startTime;
    double Deadline;
};
/***
 * 工人状态的数据结构
 */
struct CURRENT_WORKERS_GROUP
{
    WORKER worker;
    int Original_Local;
    bool sign = true;
};

/***
 * 任务状态的数据结构
 */
struct CURRENT_TASK_GROUP
{
    TASK task;
    int Original_Local;
    bool sign = true;
};

struct CURRENT_WORKER_STATE
{
    vector<double> current_worker_subTrajectoryDis;
    double current_alltaskCost = 0;
    double workerAD_orig;
    vector<CURRENT_TASK_GROUP> matched_task;
    vector<int> detour_poi;
    vector<double> matched_MaxDistanceTask_orig;
};

struct CURRENT_TASK_STATE
{
    double MaxDistanceTask_orig;
    int poi;
    double current_task_detour_distance;
};

class Basic_information
{
private:
    /* data */
public:
    double c;
    double speed;
    int Number_Task;
    int Number_Worker;
    double rangeX;
    int Capacity;
    int Wmax;
    double Tmax;
    bool Satisfaction_sign;
    int dataOption;
    vector<WORKER> global_workers;
    vector<TASK> global_tasks;
    vector<double> global_MaxDistanceTask;
    vector<vector<int>> global_POI;
    vector<vector<int>> global_CT_Worker;
    vector<vector<double>> global_detour_distance;
    vector<vector<double>> global_Worker_subTrajectoryDis;
    vector<double> global_Sumdis;
    vector<vector<pair<int, double>>> global_PT;
    vector<vector<pair<int, double>>> global_PW;
    vector<vector<double>> worknextMaxDistanceTask;
    vector<int> nextWorkSet;

    Basic_information(/* args */);

    /***
     * 速度
     */
    Basic_information(double c_, double speed_, int Number_Task_, int Number_Worker_, int Capacity_,
                      int Wmax_, double Tmax_, bool Satisfaction_sign_,
                      int dataOption_) : c(c_), speed(speed_), Number_Task(Number_Task_),
                                         Number_Worker(Number_Worker_), Capacity(Capacity_),
                                         Wmax(Wmax_), Tmax(Tmax_), Satisfaction_sign(Satisfaction_sign_),
                                         dataOption(dataOption_)
    {
        global_workers.clear();
        global_tasks.clear();
        global_POI.clear();
        global_CT_Worker.clear();
        global_detour_distance.clear();
        global_Worker_subTrajectoryDis.clear();
        global_Sumdis.clear();
        global_PT.clear();
        global_PW.clear();
        global_MaxDistanceTask.clear();

        global_workers.resize(Number_Worker);
        global_tasks.resize(Number_Task);
        global_POI.resize(Number_Task, vector<int>(Number_Worker, 0));
        global_CT_Worker.resize(Number_Worker);
        global_detour_distance.resize(Number_Task, vector<double>(Number_Worker, 0.0));
        global_Worker_subTrajectoryDis.resize(Number_Worker);
        global_Sumdis.resize(Number_Worker, 0.0);
        global_PT.resize(Number_Task);
        global_PW.resize(Number_Worker);
        global_MaxDistanceTask.resize(Number_Task);
        worknextMaxDistanceTask.clear();
        nextWorkSet.clear();
    };
    /***
     * 五类函数
     * 1、打印、不需要改变的
     * 2、距离计算等函数，不需要改变的
     * 3、生成数据，不需要改变的
     * 4、需要改变的
     * 5、展示最终结果
     */
    void print_info();
    void print_groupWork(vector<CURRENT_WORKERS_GROUP> selected_workers);
    void print_groupTasks_startTime_simple();
    void print_groupTasks_addEndTime();
    void showTask(vector<TASK> &task);
    void showWorker(vector<WORKER> &worker);
    void ShowCTMatching(vector<int> CT_Worker[], int current_Number_Worker);
    void ShowCTMatching(const char *filename, vector<vector<int>> &CT_Worker, int current_Number_Worker);
    void ShowCTMatching(vector<vector<int>> &CT_Worker, int current_Number_Workers);
    void ShowMatching(vector<pair<int, int>> &Matching);
    void printf_Satisfaction_Results(string alg_name, double run_time);
    void printf_Satisfaction_Results_num(string alg_name, double run_time, int test_range, int test_task, int test_worker);
    void printf_Results_to_txt(double task_satis_results, double worker_satis_results, string name, string alg_name, double run_time);
    void printf_Results_to_txt_num(double task_satis_results, double worker_satis_results, string sati_name,
                                   string alg_name, double run_time, int test_range, int test_task, int test_worker);
    double Caculate_mindist_global(int workerid, int taskid, vector<vector<int>> &poi);
    double Caculate_mindist(int workerid, int taskid, vector<int> poi[], vector<CURRENT_TASK_GROUP> &task, vector<CURRENT_WORKERS_GROUP> &worker);
    double rad(double d);
    double GetDistance(double lat1, double lng1, double lat2, double lng2);
    void computeMaxDitanceTask(double MaxDistanceTask[], vector<CURRENT_TASK_GROUP> currentTask, double current_window_endTime);
    void Caculate_Sumdist_Trajectory(vector<double> &Sumdis, vector<vector<double>> &global_Worker_subTrajectoryDis, vector<WORKER> &worker);
    int Compute_global_PTPW_Group(vector<vector<pair<int, double>>> &PT, vector<vector<pair<int, double>>> &PW);
    void Initialize_group(vector<double> &AD, vector<double> &Sumdis, vector<vector<double>> &current_Group_worker_subTrajectoryDis, int current_workID, double startTime, vector<CURRENT_WORKERS_GROUP> &current_workerGroup);
    void determine_Window_Task_Timeout(std::vector<CURRENT_TASK_GROUP> &taskList, double &nowTime);
    void groupWork_according_TaskGroup(vector<WORKER> &workers, vector<double> &Sumdis, vector<CURRENT_WORKERS_GROUP> &current_workerGroup,
                                       vector<double> &current_Group_workerSumdis, double nowTime, int current_workID, vector<double> &current_Group_workerAD,
                                       vector<vector<double>> &global_Worker_subTrajectoryDis, vector<vector<double>> &current_Group_worker_subTrajectoryDis);
    void UpdateTaskDeadline(vector<CURRENT_TASK_GROUP> &current_taskGroup, int workerid, int taskid, vector<double> current_detour_distance[], vector<int> CT_Worker[], vector<int> poi[], double MaxDistanceTask[], double current_task_NeedTime);
    void updata_current_Info(vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, vector<double> &current_Group_workerAD, vector<vector<double>> &current_Group_worker_subTrajectoryDis, vector<double> &current_Group_workerSumdis);

    double Caculate_Worker_Satisfaction_avg(vector<vector<int>> &CT_Worker, vector<vector<pair<int, double>>> &PW);
    double Caculate_Worker_Satisfaction_sum(vector<vector<int>> &CT_Worker, vector<vector<pair<int, double>>> &PW);
    double Caculate_Task_Satisfaction_sum(vector<vector<int>> &CT_Worker, vector<vector<pair<int, double>>> &PT);
    double Caculate_Task_Satisfaction_avg(vector<vector<int>> &CT_Worker, vector<vector<pair<int, double>>> &PT);

    void Get_Trajectory_locations(vector<WORKER> &worker);
    void Prodece_Worker_endTime_range_score(vector<WORKER> &worker, vector<double> &Sumdis, double endtimeX, double rangeX, double scoreX);
    void ReadLocationForTask(vector<TASK> &task);
    void Prodece_Task_Reward_Minscore_Deadline(vector<TASK> &task);
    void begin_Algorithm(string alg_num);

    void match_WorkerTask_Greedy(vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, double current_window_endTime, vector<double> &current_Group_workerAD, vector<vector<double>> &current_Group_worker_subTrajectoryDis);
    void Grouping_Framework_Greedy(vector<TASK> &tasks, vector<WORKER> &workers, double Wmax, int Tmax, vector<double> &Sumdis, vector<vector<double>> &global_Worker_subTrajectoryDis);
    int FindLatestWorkerNew_Greedy(int taskid, vector<int> &current_Group_worker_AW, vector<double> &current_Group_workerAD, vector<double> current_detour_distance[], vector<vector<double>> &current_Group_worker_subTrajectoryDis, vector<int> CT_Worker[], double MaxDistanceTask[], double *current_task_NeedTime, vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, vector<int> poi[], double nowtime);
    bool CurrentTask_Satisfy(vector<CURRENT_TASK_GROUP> &current_taskGroup, int workerid, int taskid, vector<double> &AD, vector<double> current_detour_distance[], vector<vector<double>> &current_Worker_subTrajectoryDis, vector<int> CT_Worker[], vector<int> poi[], double MaxDistanceTask[], double *current_task_NeedTime, double nowtime);
    bool CurrentTask_Satisfy_TSDA(vector<CURRENT_TASK_GROUP> &current_taskGroup, int workerid, int taskid, vector<double> &AD, vector<double> current_detour_distance[], vector<vector<double>> &current_Worker_subTrajectoryDis, vector<vector<int>> &CT_Worker, vector<int> poi[], double MaxDistanceTask[], double *current_task_NeedTime, double nowtime);

    void Grouping_Framework_WorkerBatch(vector<TASK> &tasks, vector<WORKER> &workers, double Wmax, int Tmax, vector<double> &Sumdis, vector<vector<double>> &global_Worker_subTrajectoryDis);
    void match_WorkerTask_workerBatch(vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, double current_window_endTime, vector<double> &current_Group_workerAD, vector<vector<double>> &current_Group_worker_subTrajectoryDis);
    void Compute_PTPW_Group_workerBatch(vector<vector<pair<int, double>>> &current_PT, vector<vector<pair<int, double>>> &current_PW, vector<double> current_detour_distance[], vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, vector<int> current_poi[]);
    void iterator_Match_WorkBatch(vector<CURRENT_WORKERS_GROUP> &current_workerGroup, vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<vector<pair<int, double>>> &current_PW, vector<vector<pair<int, double>>> &current_PT,
                                  vector<double> &current_Group_workerAD, vector<double> current_detour_distance[], vector<vector<double>> &current_Group_worker_subTrajectoryDis,
                                  vector<int> CT_Worker[], double MaxDistanceTask[], double *current_task_NeedTime, vector<int> current_poi[], double nowtime);
    //  4.3.1 TPPG分批处理函数
    int FindPreferedWorkerNew_TPPG(int taskid, vector<int> &current_Group_worker_AW, vector<double> &current_Group_workerAD, vector<double> current_detour_distance[], vector<vector<double>> &current_Group_worker_subTrajectoryDis, vector<int> CT_Worker[], double MaxDistanceTask[], double *current_task_NeedTime, vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, vector<int> poi[], double nowtime);

    void Grouping_Framework_TPPG(vector<TASK> &tasks, vector<WORKER> &workers, double Wmax, int Tmax, vector<double> &Sumdis, vector<vector<double>> &global_Worker_subTrajectoryDis);

    void match_WorkerTask_TPPG(vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, double current_window_endTime, vector<double> &current_Group_workerAD, vector<vector<double>> &current_Group_worker_subTrajectoryDis);

    //  4.3.1.1 TPPG-DAG分批处理函数
    void Grouping_Framework_TPPG_DAG(vector<TASK> &tasks, vector<WORKER> &workers, double Wmax, int Tmax, vector<double> &Sumdis,
                                     vector<vector<double>> &global_Worker_subTrajectoryDis);

    void match_WorkerTask_TPPG_DAG(vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup,
                                   double current_window_endTime, vector<double> &current_Group_workerAD,
                                   vector<vector<double>> &current_Group_worker_subTrajectoryDis);

    int FindPreferedWorkerNew_TPPG_DAG(int worker_start_number, int taskid, vector<int> &current_Group_worker_AW, vector<double> &current_Group_workerAD,
                                       vector<double> current_detour_distance[],
                                       vector<vector<double>> &current_Group_worker_subTrajectoryDis, vector<int> CT_Worker[],
                                       double MaxDistanceTask[], double *current_task_NeedTime,
                                       vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup,
                                       vector<int> poi[], double nowtime);

    //  4.3.2 TPPG_Batch分批处理函数
    int FindPreferedWorkerNew_TPPG_Batch(int taskid, vector<int> &current_Group_worker_AW, vector<double> &current_Group_workerAD, vector<double> current_detour_distance[], vector<vector<double>> &current_Group_worker_subTrajectoryDis, vector<int> CT_Worker[], double MaxDistanceTask[], double *current_task_NeedTime, vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, vector<int> poi[], double nowtime);

    void Grouping_Framework_TPPG_Batch(vector<TASK> &tasks, vector<WORKER> &workers, double Wmax, int Tmax, vector<double> &Sumdis, vector<vector<double>> &global_Worker_subTrajectoryDis);

    void match_WorkerTask_TPPG_Batch(vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, double current_window_endTime, vector<double> &current_Group_workerAD, vector<vector<double>> &current_Group_worker_subTrajectoryDis);

    void Grouping_Framework_TSDA(vector<TASK> &tasks, vector<WORKER> &workers, double Wmax, int Tmax, vector<double> &Sumdis, vector<vector<double>> &global_Worker_subTrajectoryDis);
    void match_WorkerTask_TSDA(vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, double current_window_endTime, vector<double> &current_Group_workerAD, vector<vector<double>> &current_Group_worker_subTrajectoryDis);
    vector<pair<int, double>>::iterator IFTaskExist(int workerid, int taskid, vector<vector<pair<int, double>>> &PW);
    void Update_AD1(int workerid, int taskid, vector<double> &AD, vector<double> current_detour_distance[]);
    void Update_AD2(int workerid, int taskid, int MinReplaceTask, vector<double> &AD, vector<double> detour_distance[]);
    void UpdateTaskDeadline_TSDA(vector<CURRENT_TASK_GROUP> &current_taskGroup, bool replace, int replaceid, int workerid, int taskid, vector<double> current_detour_distance[], vector<vector<int>> &CT_Worker, vector<int> poi[], double MaxDistanceTask[], double current_task_NeedTime);
    int FindReplaceTaskNew_TSDA(int workerid, int taskid, vector<vector<pair<int, double>>> &PW, vector<double> &AD, vector<double> detour_distance[], vector<vector<double>> &Worker_subTrajectoryDis, vector<vector<int>> &CT_Worker, vector<int> poi[], double MaxDistanceTask[], double *current_task_NeedTime, vector<CURRENT_TASK_GROUP> &current_taskGroup, double nowtime);
    bool IfReplace(int workerid, int taskid, int assignedtaskid, vector<double> &AD, vector<double> detour_distance[], vector<vector<double>> &Worker_subTrajectoryDis, vector<vector<int>> &CT_Worker, vector<int> poi[], double MaxDistanceTask[], double *current_task_NeedTime, double beforesumDis, vector<CURRENT_TASK_GROUP> &current_taskGroup, double nowtime);

    void Grouping_Framework_WPPG(vector<TASK> &tasks, vector<WORKER> &workers, double Wmax, int Tmax, vector<double> &Sumdis, vector<vector<double>> &global_Worker_subTrajectoryDis);
    void match_WorkerTask_WPPG(vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, double current_window_endTime, vector<double> &current_Group_workerAD, vector<vector<double>> &current_Group_worker_subTrajectoryDis);
    void ComputePWforAT_WPPG(int workerid, vector<vector<pair<int, double>>> &PW, vector<int> &AT, vector<double> current_detour_distance[], vector<int> current_poi[], vector<CURRENT_WORKERS_GROUP> &current_workerGroup, vector<CURRENT_TASK_GROUP> &current_taskGroup);

    void Grouping_Framework_WSDA(vector<TASK> &tasks, vector<WORKER> &workers, double Wmax, int Tmax, vector<double> &Sumdis, vector<vector<double>> &global_Worker_subTrajectoryDis);
    void match_WorkerTask_WSDA(vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, double current_window_endTime, vector<double> &current_Group_workerAD, vector<vector<double>> &current_Group_worker_subTrajectoryDis);
    void UpdateTaskDeadline_WSDA(vector<CURRENT_TASK_GROUP> &current_taskGroup, bool replace, int replaceWorkid, int workerid, int taskid, vector<double> current_detour_distance[], vector<vector<int>> &CT_Worker, vector<int> poi[], double MaxDistanceTask[], double current_task_NeedTime);
    void whole_Greedy_Framework();
    void erase_Task_worker_Timeout(std::vector<CURRENT_TASK_GROUP> &taskList, vector<CURRENT_WORKERS_GROUP> &workerList, double nowTime, vector<CURRENT_WORKER_STATE> &current_workerState, vector<CURRENT_TASK_STATE> &current_taskState);
    void match_Whole(vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, double current_Time, vector<CURRENT_WORKER_STATE> &workStates, vector<CURRENT_TASK_STATE> &taskStates, bool sign);
    double Caculate_mindist_whole(int global_workerid, int current_taskid, vector<CURRENT_TASK_GROUP> &task, vector<CURRENT_TASK_STATE> &taskState);
    bool CurrentTask_Satisfy_whole(vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, int current_taskid, vector<CURRENT_TASK_STATE> &taskStates, vector<CURRENT_WORKER_STATE> &workStates, double current_Time, int current_workerid, double *current_task_NeedTime);
    void UpdateTaskDeadline_whole(int current_workerid, int taskid, vector<CURRENT_WORKER_STATE> &workStates, vector<CURRENT_TASK_STATE> &taskStates, double current_task_NeedTime, vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup);
    void time_random_Framework();
    void time_random_match(vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, double current_Time, vector<CURRENT_WORKER_STATE> &workStates, vector<CURRENT_TASK_STATE> &taskStates, bool sign);
    void Grouping_Framework_ReverseDA(vector<TASK> &tasks, vector<WORKER> &workers, double Wmax, int Tmax, vector<double> &Sumdis, vector<vector<double>> &global_Worker_subTrajectoryDis);
    void match_WorkerTask_ReverseDA(vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, double current_window_endTime, vector<double> &current_Group_workerAD, vector<vector<double>> &current_Group_worker_subTrajectoryDis);

    void Grouping_Framework_AlternateDA(vector<TASK> &tasks, vector<WORKER> &workers, double Wmax, int Tmax, vector<double> &Sumdis, vector<vector<double>> &global_Worker_subTrajectoryDis);
    void match_WorkerTask_AlternateDA(vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, double current_window_endTime, vector<double> &current_Group_workerAD, vector<vector<double>> &current_Group_worker_subTrajectoryDis);
    void whole_random_Framework();
    void random_match_Whole(vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, double current_Time, vector<CURRENT_WORKER_STATE> &workStates, vector<CURRENT_TASK_STATE> &taskStates, bool sign);
    void Initialize_group_workNext(vector<double> &AD, vector<double> &Sumdis, vector<vector<double>> &current_Group_worker_subTrajectoryDis, int current_workID, double startTime, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, int ctwSize);
    void groupWork_according_TaskGroup_workNext(vector<WORKER> &workers, vector<double> &Sumdis, vector<CURRENT_WORKERS_GROUP> &current_workerGroup,
                                                vector<double> &current_Group_workerSumdis, double current_window_startTime, double nowTime, int current_workID, vector<double> &current_Group_workerAD,
                                                vector<vector<double>> &global_Worker_subTrajectoryDis, vector<vector<double>> &current_Group_worker_subTrajectoryDis);
    void UpdateTaskDeadline_workNext(vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, int workerid, int taskid, vector<double> current_detour_distance[], vector<int> CT_Worker[], vector<int> poi[], double MaxDistanceTask[], double current_task_NeedTime);
    void updata_current_Info_workNext(vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, vector<double> &current_Group_workerAD, vector<vector<double>> &current_Group_worker_subTrajectoryDis, vector<double> &current_Group_workerSumdis);

    void match_WorkerTask_Greedy_workNext(vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, double current_window_endTime, vector<double> &current_Group_workerAD, vector<vector<double>> &current_Group_worker_subTrajectoryDis);
    void Grouping_Framework_Greedy_workNext(vector<TASK> &tasks, vector<WORKER> &workers, double Wmax, int Tmax, vector<double> &Sumdis, vector<vector<double>> &global_Worker_subTrajectoryDis);
    int FindLatestWorkerNew_Greedy_workNext(int taskid, vector<int> &current_Group_worker_AW, vector<double> &current_Group_workerAD, vector<double> current_detour_distance[], vector<vector<double>> &current_Group_worker_subTrajectoryDis, vector<int> CT_Worker[], double MaxDistanceTask[], double *current_task_NeedTime, vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, vector<int> poi[], double nowtime);
    bool CurrentTask_Satisfy_workNext(vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, int workerid, int taskid, vector<double> &AD, vector<double> current_detour_distance[], vector<vector<double>> &current_Worker_subTrajectoryDis, vector<int> CT_Worker[], vector<int> poi[], double MaxDistanceTask[], double *current_task_NeedTime, double nowtime);
    bool CurrentTask_Satisfy_TSDA_workNext(vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, int workerid, int taskid, vector<double> &AD, vector<double> current_detour_distance[], vector<vector<double>> &current_Worker_subTrajectoryDis, vector<vector<int>> &CT_Worker, vector<int> poi[], double MaxDistanceTask[], double *current_task_NeedTime, double nowtime);

    void Grouping_Framework_WorkerBatch_workNext(vector<TASK> &tasks, vector<WORKER> &workers, double Wmax, int Tmax, vector<double> &Sumdis, vector<vector<double>> &global_Worker_subTrajectoryDis);
    void match_WorkerTask_workerBatch_workNext(vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, double current_window_endTime, vector<double> &current_Group_workerAD, vector<vector<double>> &current_Group_worker_subTrajectoryDis);
    void Compute_PTPW_Group_workerBatch_workNext(vector<vector<pair<int, double>>> &current_PT, vector<vector<pair<int, double>>> &current_PW, vector<double> current_detour_distance[], vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, vector<int> current_poi[]);
    void iterator_Match_WorkBatch_workNext(vector<CURRENT_WORKERS_GROUP> &current_workerGroup, vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<vector<pair<int, double>>> &current_PW, vector<vector<pair<int, double>>> &current_PT,
                                           vector<double> &current_Group_workerAD, vector<double> current_detour_distance[], vector<vector<double>> &current_Group_worker_subTrajectoryDis,
                                           vector<int> CT_Worker[], double MaxDistanceTask[], double *current_task_NeedTime, vector<int> current_poi[], double nowtime);
    int FindPreferedWorkerNew_TPPG_workNext(int taskid, vector<int> &current_Group_worker_AW, vector<double> &current_Group_workerAD, vector<double> current_detour_distance[], vector<vector<double>> &current_Group_worker_subTrajectoryDis, vector<int> CT_Worker[], double MaxDistanceTask[], double *current_task_NeedTime, vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, vector<int> poi[], double nowtime);

    void Grouping_Framework_TPPG_workNext(vector<TASK> &tasks, vector<WORKER> &workers, double Wmax, int Tmax, vector<double> &Sumdis, vector<vector<double>> &global_Worker_subTrajectoryDis);

    void match_WorkerTask_TPPG_workNext(vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, double current_window_endTime, vector<double> &current_Group_workerAD, vector<vector<double>> &current_Group_worker_subTrajectoryDis);

    //  4.3.2 TPPG_Batch分批处理函数
    int FindPreferedWorkerNew_TPPG_Batch_workNext(int taskid, vector<int> &current_Group_worker_AW, vector<double> &current_Group_workerAD, vector<double> current_detour_distance[], vector<vector<double>> &current_Group_worker_subTrajectoryDis, vector<int> CT_Worker[], double MaxDistanceTask[], double *current_task_NeedTime, vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, vector<int> poi[], double nowtime);

    void Grouping_Framework_TPPG_Batch_workNext(vector<TASK> &tasks, vector<WORKER> &workers, double Wmax, int Tmax, vector<double> &Sumdis, vector<vector<double>> &global_Worker_subTrajectoryDis);

    void match_WorkerTask_TPPG_Batch_workNext(vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, double current_window_endTime, vector<double> &current_Group_workerAD, vector<vector<double>> &current_Group_worker_subTrajectoryDis);

    void Grouping_Framework_TSDA_workNext(vector<TASK> &tasks, vector<WORKER> &workers, double Wmax, int Tmax, vector<double> &Sumdis, vector<vector<double>> &global_Worker_subTrajectoryDis);
    void match_WorkerTask_TSDA_workNext(vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, double current_window_endTime, vector<double> &current_Group_workerAD, vector<vector<double>> &current_Group_worker_subTrajectoryDis);
    void UpdateTaskDeadline_TSDA_workNext(vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, bool replace, int replaceid, int workerid, int taskid, vector<double> current_detour_distance[], vector<vector<int>> &CT_Worker, vector<int> poi[], double MaxDistanceTask[], double current_task_NeedTime);
    int FindReplaceTaskNew_TSDA_workNext(int workerid, int taskid, vector<vector<pair<int, double>>> &PW, vector<double> &AD, vector<double> detour_distance[], vector<vector<double>> &Worker_subTrajectoryDis, vector<vector<int>> &CT_Worker, vector<int> poi[], double MaxDistanceTask[], double *current_task_NeedTime, vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, double nowtime);
    bool IfReplace_workNext(int workerid, int taskid, int assignedtaskid, vector<double> &AD, vector<double> detour_distance[], vector<vector<double>> &Worker_subTrajectoryDis, vector<vector<int>> &CT_Worker, vector<int> poi[], double MaxDistanceTask[], double *current_task_NeedTime, double beforesumDis, vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, double nowtime);

    void Grouping_Framework_WPPG_workNext(vector<TASK> &tasks, vector<WORKER> &workers, double Wmax, int Tmax, vector<double> &Sumdis, vector<vector<double>> &global_Worker_subTrajectoryDis);
    void match_WorkerTask_WPPG_workNext(vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, double current_window_endTime, vector<double> &current_Group_workerAD, vector<vector<double>> &current_Group_worker_subTrajectoryDis);
    void ComputePWforAT_WPPG_workNext(int workerid, vector<vector<pair<int, double>>> &PW, vector<int> &AT, vector<double> current_detour_distance[], vector<int> current_poi[], vector<CURRENT_WORKERS_GROUP> &current_workerGroup, vector<CURRENT_TASK_GROUP> &current_taskGroup);

    void Grouping_Framework_WSDA_workNext(vector<TASK> &tasks, vector<WORKER> &workers, double Wmax, int Tmax, vector<double> &Sumdis, vector<vector<double>> &global_Worker_subTrajectoryDis);
    void match_WorkerTask_WSDA_workNext(vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, double current_window_endTime, vector<double> &current_Group_workerAD, vector<vector<double>> &current_Group_worker_subTrajectoryDis);
    void UpdateTaskDeadline_WSDA_workNext(vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, bool replace, int replaceWorkid, int workerid, int taskid, vector<double> current_detour_distance[], vector<vector<int>> &CT_Worker, vector<int> poi[], double MaxDistanceTask[], double current_task_NeedTime);

    void Grouping_Framework_ReverseDA_workNext(vector<TASK> &tasks, vector<WORKER> &workers, double Wmax, int Tmax, vector<double> &Sumdis, vector<vector<double>> &global_Worker_subTrajectoryDis);
    void match_WorkerTask_ReverseDA_workNext(vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, double current_window_endTime, vector<double> &current_Group_workerAD, vector<vector<double>> &current_Group_worker_subTrajectoryDis);

    void Grouping_Framework_AlternateDA_workNext(vector<TASK> &tasks, vector<WORKER> &workers, double Wmax, int Tmax, vector<double> &Sumdis, vector<vector<double>> &global_Worker_subTrajectoryDis);
    void match_WorkerTask_AlternateDA_workNext(vector<CURRENT_TASK_GROUP> &current_taskGroup, vector<CURRENT_WORKERS_GROUP> &current_workerGroup, double current_window_endTime, vector<double> &current_Group_workerAD, vector<vector<double>> &current_Group_worker_subTrajectoryDis);

    int GetIndex_PT(int workerid, int taskid, vector<vector<pair<int, double>>> &PT);
    int GetIndex_PW(int workerid, int taskid, vector<vector<pair<int, double>>> &PW);
};

bool cmp_task_start(const TASK &t1, const TASK &t2)
{
    return t1.startTime < t2.startTime;
}
bool cmp_worker_start(const WORKER &w1, const WORKER &w2)
{
    return w1.startTime < w2.startTime;
}
bool cmp_worker_start1(const pair<WORKER, int> &pairs1, const pair<WORKER, int> &pairs2)
{
    return pairs1.first.startTime < pairs2.first.startTime;
}
/**
 * 混合排序用于全局贪心匹配
 */
void sort_hybrid(vector<TASK> &tasks, vector<WORKER> &workers, vector<pair<WORKER, TASK>> &hybird_datasets)
{

    int i = 0, j = 0;
    while (i < workers.size() && j < tasks.size())
    {
        if (workers[i].startTime <= tasks[j].startTime)
        {
            // cout << "workers: " << workers[i].startTime << " " << workers[i].endTime << endl;
            hybird_datasets.push_back({workers[i], {}});
            i++;
        }
        else
        {
            // cout << "tasks: " << tasks[j].startTime << " " << tasks[j].Deadline << endl;
            hybird_datasets.push_back({{}, tasks[j]});
            j++;
        }
    }
    while (i < workers.size())
    {
        hybird_datasets.push_back({workers[i], {}});
        i++;
    }
    while (j < tasks.size())
    {
        hybird_datasets.push_back({{}, tasks[j]});
        j++;
    }
}
void sortWork(vector<WORKER> &worker, vector<double> &Sumdis, vector<double> &global_Sumdis, vector<vector<double>> &Worker_subTrajectoryDis, vector<vector<double>> &global_Worker_subTrajectoryDis)
{
    vector<pair<WORKER, int>> pairs(worker.size());
    for (int i = 0; i < worker.size(); ++i)
    {
        pairs[i] = {worker[i], i};
    }

    sort(pairs.begin(), pairs.end(), cmp_worker_start1);

    for (int i = 0; i < pairs.size(); ++i)
    {
        global_Worker_subTrajectoryDis[i] = Worker_subTrajectoryDis[pairs[i].second];
        global_Sumdis[i] = Sumdis[pairs[i].second];
        worker[i] = pairs[i].first;
        // cout << "工人" << i << " 时间:（" << worker[i].startTime << "," << worker[i].endTime << "), 距离：" << global_Sumdis[i] << endl;
    }
    Sumdis.clear();
    Worker_subTrajectoryDis.clear();
}

bool isTaskExpired(const TASK &task, double t)
{
    return task.Deadline < t;
}

bool cmp(pair<int, double> a, pair<int, double> b)
{
    return a.second > b.second;
}
bool optionDataset(int dataOption, Basic_information &info, vector<vector<double>> &global_Worker_subTrajectoryDis, vector<double> &Sumdis, double endtimeX);

#endif // _BASIC_INFORMATION_H_