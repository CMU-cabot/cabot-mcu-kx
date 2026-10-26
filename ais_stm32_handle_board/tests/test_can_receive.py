import os
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[1]

class ReceiveTests(unittest.TestCase):
    def test_off_on_and_target_arrival_does_not_lose_on(self):
        source=Path(os.environ.get('HANDLE_TEST_SOURCE',ROOT/'ais_stm32_handle_board.ino')).read_text()
        task=source[source.index('void task_read('):source.index('void process_message(')]
        harness=r'''
#include <cstdint>
#include <stdexcept>
#include <cstdio>
struct can_frame {uint32_t can_id; uint8_t can_dlc,data[8];};
int semaphoreCanIO=1,semaphoreCanISR=2,portMAX_DELAY=0,task_read_count=0;
void xSemaphoreTake(int s,int){if(s==2)throw 1;}
void xSemaphoreGive(int){} void debug_println(const char*){}
bool torque=false; int position=2048; int reads=0;
void process_message(can_frame f){if(f.can_id==0x98)torque=f.data[0]; if(f.can_id==0x99&&torque)position=1900;}
struct MCP2515 {
 enum {CANINTF_RX0IF=1,CANINTF_RX1IF=2,RXB0=0,RXB1=1,ERROR_OK=0};
 bool occupied[2]={true,false}; bool injected=false;
 can_frame frames[2]={{0x98,1,{1}},{}};
 void arrive(){if(injected)return; injected=true; int n=occupied[0]?1:0; frames[n]={0x99,4,{0}};occupied[n]=true;}
 int getInterrupts(){return occupied[0]|(occupied[1]<<1);}
 void clearInterrupts(){occupied[0]=occupied[1]=false;arrive();}
 int readMessage(int n,can_frame* f){arrive();*f=frames[n];occupied[n]=false;++reads;return ERROR_OK;}
}mcp2515;
''' +task+r'''
int main(){try{task_read(nullptr);}catch(int){} if(!torque||position!=1900){fprintf(stderr,"OFF->ON followed by target lost ON; no motion\n");return 1;}return 0;}
'''
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp);(p/'t.cpp').write_text(harness)
            r=subprocess.run(['g++','-std=c++11',str(p/'t.cpp'),'-o',str(p/'t')],capture_output=True,text=True)
            self.assertEqual(r.returncode,0,r.stderr)
            r=subprocess.run([str(p/'t')],capture_output=True,text=True)
            self.assertEqual(r.returncode,0,r.stderr)
