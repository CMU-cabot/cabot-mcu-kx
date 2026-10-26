from pathlib import Path
import subprocess,tempfile,unittest
ROOT=Path(__file__).resolve().parents[1]
class ResetContextTests(unittest.TestCase):
    def test_mcu_reset_preserves_newest_off_and_target(self):
        source=(ROOT/'ais_stm32_handle_board.ino').read_text()
        start=source.index('void reset_handle_preserving_request()')
        end=source.index('\nvoid debug_println',start)
        function=source[start:end]
        code=r'''
#include <cstdint>
#include <cassert>
#include "servo_monitor.h"
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
void enable_recovery_backup(){} void __DSB(){}
int resets=0; void NVIC_SystemReset(){++resets;}
struct Registers {uint32_t BKP1R=0,BKP2R=0;} regs;
Registers* RTC=&regs;
constexpr uint32_t SERVO_RESET_BUDGET=0x53525631, SERVO_RESUME_CONTEXT=0x53520000;
ServoMonitor servo_monitor;
bool onoff_recived=true; uint8_t buff_en[1]={0}; uint32_t servo_angle=1900;
''' +function+r'''
int main(){
 servo_monitor.desired=true; reset_handle_preserving_request();
 assert(resets==1 && regs.BKP1R==SERVO_RESET_BUDGET);
 assert((regs.BKP2R & 0xffffe000UL)==SERVO_RESUME_CONTEXT);
 assert((regs.BKP2R & 0x1000)==0 && (regs.BKP2R & 0xfff)==1900);
 onoff_recived=false;servo_monitor.desired=true;servo_angle=2100;reset_handle_preserving_request();
 assert((regs.BKP2R & 0x1000)!=0 && (regs.BKP2R & 0xfff)==2100);
 servo_angle=0xffffffff;reset_handle_preserving_request();assert(resets==2);
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp);(p/'t.cpp').write_text(code)
            result=subprocess.run(['g++','-std=c++11','-I',str(ROOT),str(p/'t.cpp'),'-o',str(p/'t')],capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr)
            result=subprocess.run([str(p/'t')],capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr)
