from pathlib import Path
import subprocess
import tempfile
import unittest
ROOT=Path(__file__).resolve().parents[1]

class MonitorTests(unittest.TestCase):
    def test_uart_failure_readback_retry_and_traffic_budget(self):
        code=r'''
#include "servo_monitor.h"
#include <cassert>
struct Dxl {
 bool torque=false,fail_on=false,fail_position=false,wrong_readback=false,goal_fail=false;
 int on_calls=0,off_calls=0,err=0,packet=0,reads=0,hw=0; uint32_t position=2048;
 bool torqueOn(int){++on_calls;err=fail_on?1:0;if(!err&&!wrong_readback)torque=true;return !err;}
 bool torqueOff(int){++off_calls;err=0;torque=false;return true;}
 bool setGoalPosition(int,uint32_t goal){err=goal_fail?2:0;if(torque&&!err)position=goal;return !err;}
 float getPresentPosition(int){err=fail_position?3:0;return err?0:position;}
 int getLastLibErrCode(){return err;} int getLastStatusPacketError(){return packet;}
 int readControlTableItem(int item,int){++reads;err=0;return item==1?torque:hw;}
};
int main(){
 ServoRecovery recovery;
 for(int i=0;i<1000;i++)assert(recovery.update(false,4)==0); // hardware protection
 for(int i=0;i<1000;i++)assert(recovery.update(false,255)==0); // unknown status
 for(int i=0;i<249;i++)assert(recovery.update(false,0)==0);
 assert(recovery.update(false,0)==1);
 for(int i=0;i<249;i++)assert(recovery.update(false,0)==0);
 assert(recovery.update(false,0)==2);
 for(int i=0;i<1000;i++)assert(recovery.update(false,0)==0); // no reset loop
 ServoRecovery interrupted;
 for(int i=0;i<200;i++)assert(interrupted.update(false,0)==0);
 assert(interrupted.update(true,0)==0);
 assert(interrupted.bad_cycles==0);
 ServoMonitor m; Dxl d; uint32_t p=0;
 auto tick=[&](){return m.step(d,1,1900,1,2,p);};
 // OFF->ON; first UART write fails although position reads keep succeeding.
 m.command(false);tick();m.command(true);d.fail_on=true;tick();
 assert(p==2048 && m.remaining==2 && m.last_operation==1 && m.lib_error==1);
 d.fail_on=false;tick();assert(p==1900 && m.remaining==0 && d.on_calls==2);
 tick();assert(d.on_calls==2); // no perpetual replay
 // Permanent failure is bounded; latest OFF cancels remaining ON attempts.
 m.command(true);d.fail_on=true;tick();tick();tick();int calls=d.on_calls;tick();assert(d.on_calls==calls);
 m.command(true);tick();m.command(false);tick();assert(!d.torque && m.remaining==0);
 // After initial failures the request recovers automatically at low rate.
 d.fail_on=true;m.command(true);tick();tick();tick();calls=d.on_calls;
 for(int i=0;i<49;i++)tick();assert(d.on_calls==calls);
 d.fail_on=false;tick();assert(d.on_calls==calls+1 && !m.pending);
 // Successful UART write alone is not proof: torque readback must match.
 d.torque=false;d.fail_on=false;d.wrong_readback=true;m.command(true);tick();assert(m.remaining==2);
 d.wrong_readback=false;tick();assert(m.remaining==0 && d.torque);
 d.fail_position=true;assert(!tick());assert(m.last_operation==3 && m.lib_error==3);
 d.fail_position=false;d.goal_fail=true;tick();assert(m.last_operation==2);
 d.goal_fail=false;
 // Hardware alert bit does not invalidate valid torque/position/status payloads.
 d.packet=128;d.hw=4;d.goal_fail=false;while(m.ticks%50!=49)tick();
 assert(tick());assert(m.hardware_error==4 && m.torque_valid && m.packet_error==128);
 d.packet=0;
 // 50 Hz target commands must not turn diagnostics into a 50 Hz stream.
 ServoMonitor quiet;Dxl q;int reports=0;uint8_t out[8];
 for(int i=0;i<500;i++){quiet.rx_target++;quiet.step(q,1,2048,1,2,p);reports+=quiet.report(out);}
 assert(reports<=11 && q.reads==20);
 // Persistent changing errors cannot exceed 10 Hz; counters saturate.
 ServoMonitor noisy;Dxl n;n.fail_position=true;reports=0;
 for(int i=0;i<500;i++){noisy.step(n,1,2048,1,2,p);reports+=noisy.report(out);}
 assert(reports<=100 && noisy.failures==255);
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp);(p/'t.cpp').write_text(code)
            subprocess.run(['g++','-std=c++11','-I',str(ROOT),str(p/'t.cpp'),'-o',str(p/'t')],check=True,capture_output=True)
            r=subprocess.run([str(p/'t')],capture_output=True,text=True)
            self.assertEqual(r.returncode,0,r.stderr)
