"""Regression tests exercising the actual firmware block and build entry point."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class BuildFailureTests(unittest.TestCase):
    def test_application_failure_stops_firmware_target(self):
        # Firmware build must fail before even starting a successful bootloader build.
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp)
            shutil.copy(ROOT / 'build.sh', p / 'build.sh')
            (p / 'tools').mkdir()
            (p / 'tools/generate_version.py').write_text('pass\n')
            (p / 'bootloader').mkdir()
            stub = p / 'bootloader/build.sh'
            stub.write_text('#!/bin/bash\ntouch bootloader-started\nexit 0\n')
            stub.chmod(0o755)
            compiler = p / 'arduino-cli'
            compiler.write_text('#!/bin/bash\nexit 23\n')
            compiler.chmod(0o755)
            result = subprocess.run(['bash', 'build.sh', 'firmware'], cwd=p,
                                    env={**os.environ, 'PATH': tmp + ':' + os.environ['PATH']},
                                    capture_output=True, text=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertFalse((p / 'bootloader-started').exists())


class ServoCommandTests(unittest.TestCase):
    def test_consumes_once_and_preserves_new_command_during_io(self):
        source = (ROOT / 'ais_stm32_handle_board.ino').read_text()
        task = source[source.index('void task20ms('):source.index('void setup()')]
        start = task.index('    if(servo_enable)')
        block = task[start:task.index('    vTaskDelayUntil', start)]
        harness = r'''
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
constexpr int DXL_ID=1, ADDR_SERVO_POS=0x9a, portMAX_DELAY=0;
int semaphoreCanIO;
bool servo_enable=true;
volatile bool onoff_recived=false;
volatile unsigned char buff_en[1]={0};
uint32_t servo_angle=2047;
struct can_frame {uint32_t can_id; uint8_t can_dlc,data[8];};
struct Dxl {
 int ons=0, offs=0; bool inject=false;
 bool torqueOn(int) {++ons; if(inject){buff_en[0]=0; onoff_recived=true; inject=false;} return true;}
 bool torqueOff(int) {++offs; return true;}
 void setGoalPosition(int,uint32_t) {}
 uint32_t getPresentPosition(int) {return 2047;}
} dxl;
struct Can {void sendMessage(can_frame*) {}} mcp2515;
void xSemaphoreTake(int,int){} void xSemaphoreGive(int){} void delayMicroseconds(int){}
void tick(){can_frame sendMsg;
''' + block + r'''
}
int main(){
 buff_en[0]=1; onoff_recived=true; tick(); tick(); tick();
 if(dxl.ons!=1){fprintf(stderr,"torque ON repeated %d times\n",dxl.ons); return 1;}
 dxl.inject=true; buff_en[0]=1; onoff_recived=true; tick(); tick(); tick();
 if(dxl.ons!=2 || dxl.offs!=1){fprintf(stderr,"new torque OFF lost or repeated\n");return 2;}
 return 0;
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp); (p/'test.cpp').write_text(harness)
            subprocess.run(['g++','-std=c++11',str(p/'test.cpp'),'-o',str(p/'test')],check=True,capture_output=True)
            result=subprocess.run([str(p/'test')],capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr)
