//  File:  Task.h
//  Co-operative non-pre-emptive tasking support.  class Task is abstract base class for user defined
//  working tasks.  class TaskManager co-ordinates the tasks.  When a task is scheduled
//  to run it controls the cpu and runs to completion.  Task work is done in the virtual
//  task member function run(). 

 // Define concrete classes derived from class Task to define a useable task.  Over-ride initialization
 // member function setup().  This function will be called from task manager addTask() routine typically from
 // global setup() function called from arduino main();    
// 

#ifndef TASK_H
#define TASK_H

// #include "TaskEvent.h"

//--------------- Abstract base class for tasks
class TaskEvent;

class Task
{
  public:
    Task(const char *pName,
         float repeatTime    // in milliseconds
         );
    virtual void clockTick(unsigned long time);
    virtual void setup() = 0;
    virtual void run(unsigned long time) = 0;
    virtual void event(TaskEvent *, unsigned long time);
    const char *taskName();
    float taskRepeatSecs();
  protected:
    void runTheTask(unsigned long time);
  private:
    unsigned long m_repeatTime;
    unsigned long m_nextRunTime;
    const char* m_pName;
};

//--------------- Manager of tasks
class TaskManager
{
    struct TaskListNode
    {
      Task *m_pTask;
      TaskListNode *m_pNext;
    };
 
    public:
      TaskManager();
      void addTask(Task *pTask);
      void addTask(Task &task);
      void execute();
      void postEvent(TaskEvent *);
    private:
      TaskListNode *m_pTaskList;  // head of task list
      TaskListNode *m_pLast;      // end of list
      TaskEvent *m_pTaskEvent;
};

//--------------------------------------------
extern TaskManager TaskSystem;

#endif

