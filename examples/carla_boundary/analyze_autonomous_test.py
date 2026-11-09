"""Create a readable report and plots from a completed autonomous trial."""
import argparse
import hashlib
import json
import os
from pathlib import Path

os.environ.setdefault('MPLCONFIGDIR','/tmp/carla-matplotlib')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import cv2


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory',type=Path)
    args=parser.parse_args()
    p=args.directory.resolve()
    summary=json.loads((p/'summary.json').read_text())
    rows=[json.loads(line) for line in (p/'frames.jsonl').read_text().splitlines()]
    trials=summary['trials']
    if not trials:raise RuntimeError('No completed trials')
    fig,axes=plt.subplots(3,len(trials),figsize=(5*len(trials),8),sharex='col',squeeze=False,
                          gridspec_kw={'height_ratios':[2,2,1]})
    extra=[]
    for j,trial in enumerate(trials):
        r=[x for x in rows if x['trial']==trial['trial']]
        assert all(b['frame_id']==a['frame_id']+1 for a,b in zip(r,r[1:]))
        assert all(abs(b['timestamp_s']-a['timestamp_s']-.05)<1e-5 for a,b in zip(r,r[1:]))
        times=np.array([x['trial_elapsed_s'] for x in r])
        red=[x for x in r if x['signal']=='Red']
        first_stop=next((x for x in red if x['trial_elapsed_s']>3 and x['front_gap_m']<3.5 and x['speed_kmh']<.72),None)
        restarted=any(x['trial_elapsed_s']>first_stop['trial_elapsed_s'] and x['speed_kmh']>1 for x in red) if first_stop else None
        five=min(r,key=lambda x:abs(x['trial_elapsed_s']-5))
        extra.append({'trial':trial['trial'],'initial_error_m':r[0]['lane_error_m'],
                      'error_after_5s_m':five['lane_error_m'],'restarted_while_red_after_stop':restarted})
        red_end=max(x['trial_elapsed_s'] for x in red)
        for a in axes[:2,j]:
            a.axvspan(0,red_end,color='#ffe3e3');a.axvspan(red_end,times[-1],color='#e1f5e6');a.grid(alpha=.25)
        axes[0,j].plot(times,[x['speed_kmh'] for x in r],color='#1f5aac',lw=2)
        axes[0,j].set_title(f'Trial {trial["trial"]}: initial offset {trial["initial_offset_m"]:+.2f} m',fontweight='bold')
        axes[0,j].set_ylim(-.5,14)
        axes[0,j].text(1,12.3,'RED',color='#b51e2b');axes[0,j].text(red_end+1,12.3,'GREEN',color='#187339')
        axes[1,j].plot(times,[x['lane_error_m']*100 for x in r],color='#263b65',lw=2)
        for y in [-50,0,50]:axes[1,j].axhline(y,color='#64748b',ls='--',lw=1)
        axes[1,j].set_ylim(-55,55)
        sources=np.array([int(x['steering_source'].startswith('VISION')) for x in r])
        axes[2,j].fill_between(times,0,1,where=sources==1,color='#2468c4',step='post',label='Boundary steering')
        axes[2,j].fill_between(times,0,1,where=sources==0,color='#e6a03c',step='post',label='Map assist')
        axes[2,j].set_yticks([]);axes[2,j].set_ylim(0,1);axes[2,j].set_xlabel('Simulation seconds')
    axes[0,0].set_ylabel('Speed (km/h)');axes[1,0].set_ylabel('Route center error (cm)');axes[2,0].set_ylabel('Steering source')
    passed=sum(t['red_stop_confirmed'] and t['green_departure_confirmed'] for t in trials)
    fig.suptitle(f'Autonomous test: {passed}/{len(trials)} red stops and green departures | No manual driving',fontsize=16,fontweight='bold')
    handles,labels=axes[2,0].get_legend_handles_labels();fig.legend(handles,labels,loc='lower center',ncol=2,frameon=False)
    fig.tight_layout(rect=[0,.04,1,.95]);fig.savefig(p/'metrics.png',dpi=140);plt.close(fig)
    cap=cv2.VideoCapture(str(p/'review.mp4'));count=0
    while True:
        ok,_=cap.read()
        if not ok:break
        count+=1
    cap.release()
    assert count==summary['total_video_frames']==len(rows)
    verification={'video_frames':count,'sensor_timestamps_aligned':True,'trials':extra,
                  'source_hashes':{name:hashlib.sha256((Path(__file__).parent/name).read_bytes()).hexdigest()
                                   for name in ['boundary_controller.py','autonomous_boundary_drive.py','road_boundary_recovery/road_boundary_recovery.py']}}
    (p/'verification.json').write_text(json.dumps(verification,indent=2))
    overall=summary['overall']
    lines=[f'첨부 검출값 기반 자동주행 최종 검증 — {p.name}',
           f'판정: {"통과" if summary["all_trials_passed"] else "결과 확인 필요"}',
           '', 'CARLA 기본 autopilot OFF. 직접 throttle/brake/steer 명령으로 주행.',
           '첨부 BoundaryTracker의 경계로 목표점을 투영해 실제 조향. 원본 검출 코드는 변경하지 않음.',
           '신호는 CARLA API에서 읽음. 교차로/검출 누락은 지도 보조이며 화면에 따로 표시.',
           '', '시험: Town02_Opt 직진 교차로, 목표 12km/h, 중앙/좌우 30cm 출발.',
           '빨간불에서 정지 후 2초 유지 → 시험기가 초록불로 전환 → 자동 출발/통과.',
           '정지 거리는 CARLA 신호 정지 기준 평면에서 차량 앞쪽까지 측정.',
           '차선 오차는 1km/h 초과 이동 중 교차로를 제외한 지도 차선 중심 기준.', '']
    for t,e in zip(trials,extra):
        lines.append(f'{t["trial"]}회: 시작 {t["initial_offset_m"]:+.2f}m, 정지 {t["red_stop_confirmed"]}, 초록 출발 {t["green_departure_delay_s"]:.2f}초, 정지 여유 {t["stop_front_gap_m"]:.3f}m, 5초 후 중심 오차 {e["error_after_5s_m"]:+.3f}m')
    lines += ['',f'일반 차로 중심 절대오차: 평균 {overall["nonjunction_lane_abs_mean_m"]*100:.2f}cm / 최대 {overall["nonjunction_lane_abs_max_m"]*100:.2f}cm.',
              '최대값에는 의도적인 30cm 초기 편차가 포함됨.',
              f'이동 중 검출 기반 조향 {overall["vision_steering_percent_moving"]:.2f}% (최대 0.2초 유지 추정 포함), 나머지는 지도 보조.',
              f'충돌 {len(summary["collisions"])}회 / 차로 밖 차량영역 {overall["nonjunction_vehicle_outside_lane_frames"]}프레임 / 수동 조작 0회.',
              f'빨간불에서 정지 후 다시 가속했는지: {[e["restarted_while_red_after_stop"] for e in extra]} (False = 정지 유지).',
              f'영상 {count}프레임, {count/20:.2f}초. 프레임별 센서/차량 상태 정렬 검증 완료.',
              '', '범위: 저속 직진과 해당 교차로 조건. 영상만으로 신호 인식/교차로 주행을 검증한 결과가 아님.',
              '급커브, 복잡한 교차로, 다른 차량/보행자, 악천후/야간은 미시험.',
              '', "현재 결과 창의 '자동 시험 다시 실행' 버튼으로 반복 가능. Space: 중단, Esc: 닫기.",
              '실행: python3 autonomous_boundary_drive.py --cycles 3 --keep-open',
              '지도 보조 없이 시험: --vision-only 추가 (검출 누락 시 정지).',
              '', '영상: review.mp4 / 그래프: metrics.png / 수치: summary.json / 원시 로그: frames.jsonl',
              '단위 테스트: python3 -m unittest test_boundary_controller.py',
              'CARLA 신호 API: https://carla.readthedocs.io/en/0.9.16/python_api/#carlatrafficlight']
    (p/'검증결과.txt').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    print(json.dumps({'passed':summary['all_trials_passed'],'overall':overall,'extra':extra},indent=2))
    print(p/'검증결과.txt')


if __name__=='__main__':main()
