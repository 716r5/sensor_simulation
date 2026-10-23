from pathlib import Path
import sys, csv, json, subprocess
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
import work

rng=np.random.default_rng(42)
OUTPUT=HERE/'simulation_results'
OUTPUT.mkdir(exist_ok=True)
temp=OUTPUT/'calibration_inputs'
temp.mkdir(exist_ok=True)
bb=np.array([.03,-.02,.01]); bs=np.array([1.02,.99,1.01])
ab=np.array([.02,.01,-.03]); ass=np.array([.98,1.02,1.01])
gb=np.array([.4,-.3,.2])
def capture(name, target, n):
    path=temp/(name+'.csv')
    b=(target*bs+bb+rng.normal(0,.002,(n,3)))*16384
    a=(target*ass+ab+rng.normal(0,.004,(n,3)))/.0039
    g=(gb+rng.normal(0,.05,(n,3)))*65.6
    with path.open('w',newline='') as f:
        w=csv.writer(f);w.writerow(work.HEADER)
        for k in range(n):
            w.writerow([1,k,k*100000,*np.round(np.r_[g[k],b[k],a[k]]).astype(int),27898,23843,25.,100000.])
    return str(path)
faces=[]
for k,name in enumerate(['PX','NX','PY','NY','PZ','NZ']):
    target=np.zeros(3);target[k//2]=1 if k%2==0 else -1
    faces.append(capture(name,target,300))
cb=work.six_face(faces,'bmi'); ca=work.six_face(faces,'adxl')
still=work.load(capture('validation',np.array([0,0,1]),600))
b,a,g=work.physical(still)
b_cal=(b-cb['bias_g'])/cb['scale'];a_cal=(a-ca['bias_g'])/ca['scale']
gyro_cal=work.physical(work.load(capture('gyro_cal',np.array([0,0,1]),600)))[2].mean(axis=0)
cal_stats={'bmi_bias_error_g':float(np.max(abs(np.array(cb['bias_g'])-bb))),
           'adxl_bias_error_g':float(np.max(abs(np.array(ca['bias_g'])-ab))),
           'bmi_validation_norm_g':float(np.mean(np.linalg.norm(b_cal,axis=1))),
           'adxl_validation_norm_g':float(np.mean(np.linalg.norm(a_cal,axis=1))),
           'gyro_validation_max_mean_dps':float(np.max(abs((g-gyro_cal).mean(axis=0))))}
assert cal_stats['bmi_bias_error_g']<.001
assert cal_stats['adxl_bias_error_g']<.001
assert cal_stats['gyro_validation_max_mean_dps']<.02

# White-noise and slow linear-drift signals, same seeded noise realization.
n=18000;dt=.1;t=np.arange(n)*dt
noise=rng.normal(0,.2,n)
drift=noise+.001*t
vn=work.variances(noise,dt);vd=work.variances(drift,dt)
assert abs(vn[0,1]-.04)<.002 and abs(vn[0,2]-.04)<.002
idx=int(np.argmin(abs(vn[:,0]-10)))
variance_stats={'tau_first_s':float(vn[0,0]),'allan_first_dps2':float(vn[0,1]),
                'hadamard_first_dps2':float(vn[0,2]),'tau_comparison_s':float(vn[idx,0]),
                'allan_comparison_dps2':float(vn[idx,1])}

# Two separate known-axis motion cases. No yaw or navigation claim.
tm=np.arange(601)*dt;truth=20*np.sin(2*np.pi*tm/20)
truth_rate=20*(2*np.pi/20)*np.cos(2*np.pi*tm/20)
estimates=[];errors=[]
for axis in ['roll','pitch']:
    f=work.TiltEKF(np.array([0.,0.,1.]))
    trace=[0.]
    for k in range(1,len(tm)):
        angle=np.deg2rad(truth[k])
        accel=np.array([0.,np.sin(angle),np.cos(angle)]) if axis=='roll' else np.array([-np.sin(angle),0.,np.cos(angle)])
        accel+=rng.normal(0,.01,3)
        gyro=rng.normal(0,.2,3)
        gyro[0 if axis=='roll' else 1]+=truth_rate[k-1]
        estimate,_=f.step(np.deg2rad(gyro),accel,dt)
        assert np.linalg.eigvalsh(f.P).min()>0
        trace.append(float(estimate[0 if axis=='roll' else 1]))
    estimates.append(np.array(trace))
    errors.append(float(np.sqrt(np.mean((trace-truth)**2))))
assert max(errors)<1
stats={'seed':42,'calibration':cal_stats,'variance':variance_stats,
       'ekf_roll_rmse_deg':errors[0],'ekf_pitch_rmse_deg':errors[1]}
(OUTPUT/'simulation_metrics.json').write_text(json.dumps(stats,indent=2))

plt.rcParams.update({'font.size':9,'axes.spines.top':False,'axes.spines.right':False})
fig,ax=plt.subplots(2,2,figsize=(10,6.1))
for k,label in enumerate(['roll','pitch']):
    ax[0,k].plot(tm,truth,color='black',lw=1.3,label='reference')
    ax[0,k].plot(tm,estimates[k],color='#26759e',lw=.9,label='EKF')
    ax[0,k].set(title=label+' simulation',xlabel='time (s)',ylabel='angle (degrees)')
    ax[0,k].legend(fontsize=8)
for rows,style,label in [(vn,'-','noise only'),(vd,'--','noise + drift')]:
    ax[1,0].loglog(rows[:,0],np.sqrt(rows[:,1]),style,label=label+' Allan')
    ax[1,0].loglog(rows[:,0],np.sqrt(rows[:,2]),style,label=label+' Hadamard')
ax[1,0].set(title='stability simulation',xlabel='averaging time (s)',ylabel='deviation (degrees/s)')
ax[1,0].legend(fontsize=6.5)
vals=[np.linalg.norm(b,axis=1).mean(),np.linalg.norm(b_cal,axis=1).mean(),np.linalg.norm(a,axis=1).mean(),np.linalg.norm(a_cal,axis=1).mean()]
ax[1,1].scatter(['BMI before','BMI after','ADXL before','ADXL after'],vals,c=['#8a9da7','#26759e','#8a9da7','#26759e'],s=45,zorder=3)
ax[1,1].axhline(1,color='black',ls='--',lw=1)
ax[1,1].set(title='separate stationary validation set',ylabel='mean acceleration norm (g)',ylim=(.96,1.05))
ax[1,1].tick_params(axis='x',labelsize=7)
for a_ in ax.flat:a_.grid(alpha=.18)
fig.tight_layout(pad=1.5)
fig.savefig(OUTPUT/'simulation_plots.png',dpi=180)
plt.close(fig)

# These are generated measurements, not physical sensor recordings.
np.savetxt(OUTPUT/'tilt_simulation.csv', np.column_stack([tm, truth, estimates[0], estimates[1]]), delimiter=',', header='t_s,reference_deg,roll_ekf_deg,pitch_ekf_deg', comments='')
np.savetxt(OUTPUT/'variance_simulation.csv', np.column_stack([vn[:,0],vn[:,1],vn[:,2],vd[:,1],vd[:,2]]), delimiter=',', header='tau_s,noise_allan_dps2,noise_hadamard_dps2,drift_allan_dps2,drift_hadamard_dps2', comments='')
expected=json.loads((HERE/'expected_simulation_metrics.json').read_text())
def verify(actual, reference):
    for name, value in reference.items():
        if isinstance(value, dict):
            verify(actual[name], value)
        elif not np.isclose(actual[name], value, rtol=1e-10, atol=1e-12):
            raise AssertionError('Result differs from report reference: '+name)
verify(stats, expected)
subprocess.run([sys.executable, str(HERE/'test_software.py')],check=True)
print(json.dumps(stats,indent=2))
print('PASS: simulation metrics match the report reference')
print('Graphs and CSV files saved in', OUTPUT)
