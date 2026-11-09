"""Verify and summarize a recorded BehaviorAgent + boundary driving run."""
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

BASE=Path(__file__).resolve().parent


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory',type=Path)
    args=parser.parse_args()
    output=args.directory.resolve()
    summary=json.loads((output/'summary.json').read_text())
    rows=[json.loads(s) for s in (output/'frames.jsonl').read_text().splitlines()]
    cap=cv2.VideoCapture(str(output/'drive.mp4'))
    video_count=0
    while True:
        ok,_=cap.read()
        if not ok:break
        video_count+=1
    cap.release()
    upstream=BASE/'third_party/carla_agents'
    manifest=json.loads((upstream/'UPSTREAM.json').read_text())
    verified_upstream=all(hashlib.sha256((upstream/name).read_bytes()).hexdigest()==digest
                          for name,digest in manifest['sha256'].items())
    source_match=all(hashlib.sha256((BASE/name).read_bytes()).hexdigest()==digest
                     for name,digest in summary['source_hashes'].items())
    red=[r for r in rows if r['red_hazard']]
    visual=[r for r in rows if r['steering_source'].startswith('BOUNDARY')]
    checks={
        'video_log_summary_frame_counts_match':video_count==len(rows)==summary['frames'],
        'consecutive_camera_frame_ids':all(b['frame_id']==a['frame_id']+1 for a,b in zip(rows,rows[1:])),
        'sensor_timestamps_20hz':all(abs(b['timestamp_s']-a['timestamp_s']-.05)<1e-5 for a,b in zip(rows,rows[1:])),
        'upstream_hashes_unchanged':verified_upstream,
        'tested_code_matches_current_files':source_match,
        'red_hazard_never_accelerates':bool(red) and all(r['throttle']==0 and r['brake']>0 for r in red),
        'red_stop_and_green_departure_observed':summary['red_stops']>0 and summary['green_departures']>0,
        'multiple_turns_observed':summary['turns_completed']>=2,
        'boundary_actually_changes_steering':any(abs(r['steer']-r['upstream_steer'])>.001 for r in visual),
        'no_collision':not summary['collisions'],
        'vehicle_footprint_inside_lane_outside_junctions':summary['vehicle_outside_lane_road_frames']==0,
        'no_runtime_error':summary['error'] is None,
    }
    details={'checks':checks,'passed':all(checks.values()),'video_frames':video_count,
             'vision_changes_steering_frames':sum(abs(r['steer']-r['upstream_steer'])>.001 for r in visual),
             'red_hazard_frames':len(red),'mean_road_error_recomputed_m':float(np.mean([
                 abs(r['lane_error_m']) for r in rows if r['speed_kmh']>1 and not r['junction']]))}
    (output/'verification.json').write_text(json.dumps(details,indent=2))

    times=np.array([r['elapsed_s'] for r in rows])
    fig,axes=plt.subplots(3,1,figsize=(13,8),sharex=True,gridspec_kw={'height_ratios':[2,2,1]})
    fig.suptitle('CARLA BehaviorAgent + uploaded boundary detector | No manual driving',fontweight='bold')
    axes[0].plot(times,[r['speed_kmh'] for r in rows],color='#2563eb',lw=1.5,label='Vehicle speed')
    axes[0].plot(times,[r['target_speed_kmh'] for r in rows],color='#64748b',lw=.8,ls='--',label='Speed cap (slower in turns)')
    axes[0].fill_between(times,0,summary['speed_limit_kmh']+2,
                         where=[r['red_hazard'] for r in rows],color='#fecaca',alpha=.7,label='Red light braking')
    axes[0].set_ylabel('Speed (km/h)');axes[0].legend(loc='upper right',fontsize=8,ncol=3)
    err=np.array([r['lane_error_m'] if not r['junction'] else np.nan for r in rows])*100
    axes[1].plot(times,err,color='#0f766e',lw=1.4)
    axes[1].axhline(0,color='#94a3b8',lw=.7)
    axes[1].set_ylabel('Road center error (cm)')
    axes[1].text(.01,.95,'Junctions excluded from lane error statistics',transform=axes[1].transAxes,va='top',fontsize=8)
    is_visual=[r['steering_source'].startswith('BOUNDARY') for r in rows]
    axes[2].fill_between(times,0,1,where=is_visual,color='#2563eb',step='post',label='Boundary detector steering')
    axes[2].fill_between(times,0,1,where=np.logical_not(is_visual),color='#e6a03c',step='post',label='BehaviorAgent control')
    axes[2].set_ylim(0,1);axes[2].set_yticks([]);axes[2].set_ylabel('Control source')
    axes[2].set_xlabel('Simulation time (seconds)');axes[2].legend(loc='upper right',fontsize=8,ncol=2)
    for axis in axes[:2]:axis.grid(alpha=.2)
    fig.tight_layout();fig.savefig(output/'metrics.png',dpi=140);plt.close(fig)

    delays=[e['delay_s'] for e in summary['events'] if e['event']=='green_departure']
    lines=[f'오픈소스 자율주행 실행 검증 — {output.name}',
           f'자동 검사: {"통과" if details["passed"] else "확인 필요"}',
           'CARLA 공식 BehaviorAgent 0.9.16 (MIT) 원본을 그대로 사용하고 별도 통합 클래스로 설정.',
           '자연 신호 주기에서 실행. 시험 중 신호 색을 강제로 바꾸지 않음.',
           f'기록 {summary["elapsed_s"]:.1f}초 / {video_count}프레임 / 거리 {summary["travelled_m"]:.2f}m.',
           f'빨간불 정지 {summary["red_stops"]}회 / 초록불 출발 {summary["green_departures"]}회 / 출발 지연 {delays}초.',
           f'목적지 도달 {summary["destinations_reached"]}회 / 교차로 회전 {summary["turns_completed"]}회.',
           f'일반 차로 중심 절대오차: 평균 {summary["road_center_abs_mean_m"]*100:.2f}cm, 95백분위 {summary["road_center_abs_p95_m"]*100:.2f}cm, 최대 {summary["road_center_abs_max_m"]*100:.2f}cm.',
           '차로 통계는 속도 1km/h 초과, 지도상 교차로 바깥 프레임에서 계산. 곡선 도로 포함.',
           f'차량 영역이 차로 폭을 벗어난 일반 도로 프레임 {summary["vehicle_outside_lane_road_frames"]}개.',
           f'충돌 {len(summary["collisions"])}회 / 차선 침범 센서 이벤트 {len(summary["lane_crossing_events"])}회 / 수동 운전 {summary["manual_driving_inputs"]}회.',
           f'이동 중 첨부 검출값 기반 조향 {summary["vision_percent_moving"]:.2f}% (최대 0.2초 유지 추정 포함).',
           f'오픈소스 조향과 다른 영상 조향이 실제 적용된 프레임 {details["vision_changes_steering_frames"]}개.',
           '', '자동 검증 항목:']
    lines.extend(f'{"PASS" if passed else "FAIL"}: {name}' for name,passed in checks.items())
    lines += ['', '범위와 해석:',
              '신호 상태·경로·주변 객체는 CARLA 월드/지도 API를 사용함. 카메라만으로 모든 것을 인식하는 자율주행은 아님.',
              '첨부 카메라 검출은 적합한 직선 구간의 조향에 실제 사용. 회전·검출 누락·제동은 BehaviorAgent 담당.',
              'Town02_Opt, 낮, 저속, 다른 차량/보행자 없는 조건. 장애물 회피·야간·악천후·실차 성능은 이 기록으로 검증하지 않음.',
              '정지 횟수는 빨간불 영향 구간에서 속도 0.36km/h 미만을 1초 유지한 사건. 출발은 초록불 이후 1.8km/h 초과.',
              '회전 횟수는 교차로 진입·진출 방향이 35도 이상 변한 사건. 교차로가 아닌 곡선은 별도 가산하지 않음.',
              '검증 범위는 위 기록 구간. 녹화 종료 뒤에도 실행 창에서 자동주행은 계속됨.',
              '', '영상 drive.mp4 / 그래프 metrics.png / 상세 summary.json / 검증 verification.json / 프레임 로그 frames.jsonl']
    (output/'검증결과.txt').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    print(json.dumps(details,indent=2))
    if not details['passed']:raise SystemExit(1)


if __name__=='__main__':main()
