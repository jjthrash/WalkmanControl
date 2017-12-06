//File: Task.cpp

#include "Task.h"
#include "TaskEvent.h"
#include <Arduino.h>
//----------------
 TaskManager TaskSystem;


//-----------------------
const unsigned long MAX_LONG = 0xffffffff;



Task::Task(const char *pName,float repeatTime):m_pName(pName),m_repeatTime(1000 * repeatTime),m_nextRunTime(0)
{
     
}

const char *Task::taskName()
{
  return m_pName;
}


void Task::runTheTask(unsigned long time)
{
  run(time);
}

void Task::clockTick(unsigned long time)
    {
      long timeDiff = m_nextRunTime - time;
      if(timeDiff < 0)
      {      
      	   m_nextRunTime += m_repeatTime;
          runTheTask(time);
      }
    }

float Task::taskRepeatSecs()
{
  float retVal = (float)m_repeatTime;
  retVal /= 1000000.0;    // convert microsecs to secs
  return retVal;
}
void Task::event(TaskEvent *pEvent, unsigned long time)
{
  
}

//-------------------------
TaskManager::TaskManager():m_pTaskEvent(0)
      {
        m_pTaskList = NULL;
        m_pLast = NULL;
      }
      
void TaskManager::addTask(Task *pTask)
{
        TaskListNode *pNode = new TaskListNode;
        pNode->m_pTask = pTask;
        pNode->m_pNext = NULL;
 
        if(m_pTaskList == NULL)
        {
          m_pTaskList = pNode;
          m_pLast = pNode;
        }
        else
        {
          m_pLast->m_pNext = pNode;
          m_pLast = pNode;
          
        }
        //Serial.print("Calling setup on "); Serial.println(pTask->taskName());
        pTask->setup();
}

void TaskManager::addTask(Task &task)
{
  addTask(&task);
}

void TaskManager::postEvent(TaskEvent *pEvent)
{
  m_pTaskEvent = pEvent;
  
}

void TaskManager::execute()
      {
        TaskListNode *pListNode = m_pTaskList;
        unsigned long time = micros();
        // give each task a chance to execute
        while(pListNode!=NULL)
        {
          pListNode->m_pTask->clockTick(time);
          pListNode = pListNode->m_pNext;
        }
      }


 
