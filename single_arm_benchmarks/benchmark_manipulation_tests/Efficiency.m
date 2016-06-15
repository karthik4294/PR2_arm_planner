clear all;
all_times_dts = [];
all_times_ma = [];
all_times_ma2 = [];
all_times_rr = [];
all_exp_dts = [];
all_exp_ma = [];
all_exp_ma2 = [];
all_exp_rr = [];
all_times_rrt = [];
success_dts = [];
success_ma = [];
success_ma2 = [];
success_rr = [];
% for i=0:9
%   dts = load(['mha_stats_dts_' num2str(i) '.csv']);
  ma = load(['benn_planner_stats' '.csv']);
  ma2 = load(['benn_planner_stats' '.csv']);
%   rr = load(['mha_stats_rr_' num2str(i) '.csv']);
  rrt = load(['benn_planner_stats' '.csv']);

%   success_dts = [success_dts; dts(:,10)>0];
  success_ma = [success_ma; ma(:,10)>0];
  success_ma2 = [success_ma2; ma2(:,10)>0];
%   success_rr = [success_rr; rr(:,10)>0];

%   all_times_dts = [all_times_dts; dts(:,1)];
  all_times_ma = [all_times_ma; ma(:,1)];
  all_times_ma2 = [all_times_ma2; ma2(:,1)];
%   all_times_rr = [all_times_rr; rr(:,1)];
  all_times_rrt = [all_times_rrt; rrt(:,1)];
%   all_exp_dts = [all_exp_dts; dts(:,8)];
  all_exp_ma = [all_exp_ma; ma(:,8)];
  all_exp_ma2 = [all_exp_ma2; ma2(:,8)];
  
%   all_exp_rr = [all_exp_rr; rr(:,8)];
% end

% idx_dts = (all_times_dts >= 0) & success_dts;
idx_ma = (all_times_ma >= 0) & success_ma;
idx_ma2 = (all_times_ma2 >= 0) & success_ma2;
% idx_rr = (all_times_rr >= 0) & success_rr;
idx_rrt = (all_times_rrt >= 0);
% pause;
% dts_paths = getAllPaths(idx_dts,'dts');
% ma_paths = getAllPaths(idx_ma,'static');
% ma2_paths = getAllPaths(idx_ma2,'dynamic');
% % rr_paths = getAllPaths(idx_rr,'rr');
% rrt_paths = getAllPaths(idx_rrt,'rrt'); %uncomment after...

% success_dts = sum(idx_dts);
success_ma = sum(idx_ma);
success_ma2 = sum(idx_ma2);
% success_rr = sum(idx_rr);
success_rrt = sum(idx_rrt);

% dts_rr = idx_dts & idx_rr;
% dts_rrt = idx_dts & idx_rrt;
% ma_rr = idx_ma & idx_rr;
ma_rrt = idx_ma & idx_rrt;
ma_ma2 = idx_ma & idx_ma2;
rrt_ma2 = idx_rrt & idx_ma2;
% dts_ma = idx_dts & idx_ma;

total_success = sum(idx_ma |idx_rrt | idx_ma2);
% in_common_dts_rr = sum(dts_rr);
% in_common_dts_rrt = sum(dts_rrt);
% in_common_ma_rr = sum(ma_rr);
in_common_ma_rrt = sum(ma_rrt);
in_common_ma_ma2 = sum(ma_ma2);
in_common_rrt_ma2 = sum(rrt_ma2);
% in_common_dts_ma = sum(dts_ma);

% time_speedup_dts_rr = all_times_rr(dts_rr)./all_times_dts(dts_rr);
% time_speedup_dts_rrt = all_times_rrt(dts_rrt)./all_times_dts(dts_rrt);
% time_speedup_ma_rr = all_times_rr(ma_rr)./all_times_ma(ma_rr);
time_speedup_ma_rrt = all_times_rrt(ma_rrt)./all_times_ma(ma_rrt);
time_speedup_ma_ma2 = all_times_ma(ma_ma2)./all_times_ma2(ma_ma2);
time_speedup_rrt_ma2 = all_times_rrt(rrt_ma2)./all_times_ma2(rrt_ma2);



time_slowdown_ma_ma2 = all_times_ma2(ma_ma2)./all_times_ma(ma_ma2);
time_slowdown_rrt_ma2 = all_times_ma2(rrt_ma2)./all_times_rrt(rrt_ma2);
% time_speedup_dts_ma = all_times_ma(dts_ma)./all_times_dts(dts_ma);
% time_speedup_ma_dts = all_times_dts(dts_ma)./all_times_ma(dts_ma);

percentage_speedups_rrt = sum(time_speedup_rrt_ma2  > 1)/ length(time_speedup_rrt_ma2);
percentage_speedups_ma = sum(time_speedup_ma_ma2  > 1)/ length(time_speedup_ma_ma2);

percentage_slowdowns_rrt = sum(time_slowdown_rrt_ma2  > 1)/ length(time_slowdown_rrt_ma2);
percentage_slowdowns_ma = sum(time_slowdown_ma_ma2  > 1)/ length(time_slowdown_ma_ma2);
%add in here
% exp_speedup_dts_rr = all_exp_rr(dts_rr)./all_exp_dts(dts_rr);
% exp_speedup_ma_rr = all_exp_rr(ma_rr)./all_exp_ma(ma_rr);
exp_speedup_ma_ma2 = all_exp_ma(ma_ma2)./all_exp_ma2(ma_ma2);
exp_slowdown_ma_ma2 = all_exp_ma2(ma_ma2)./all_exp_ma(ma_ma2);
% exp_speedup_rrt_ma2 = all_exp_ma2(rrt_ma2)./all_exp_rrt(rrt_ma2);
% exp_speedup_dts_ma = all_exp_ma(dts_ma)./all_exp_dts(dts_ma);
% exp_speedup_ma_dts = all_exp_dts(dts_ma)./all_exp_ma(dts_ma);

percentage_speedups_exp_ma = sum(exp_speedup_ma_ma2  > 1)/ length(exp_speedup_ma_ma2);
percentage_slowdowns_exp_ma = sum(exp_slowdown_ma_ma2  > 1)/ length(exp_slowdown_ma_ma2);

pause
% avg_time_speedup_dts_rr = mean(time_speedup_dts_rr);
% avg_time_speedup_dts_rrt = mean(time_speedup_dts_rrt);
% avg_time_speedup_ma_rr = mean(time_speedup_ma_rr);
avg_time_speedup_ma_rrt = mean(time_speedup_ma_rrt);
avg_time_speedup_ma_ma2 = mean(time_speedup_ma_ma2);
avg_time_speedup_rrt_ma2 = mean(time_speedup_rrt_ma2);


avg_time_slowdown_ma_ma2 = mean(time_slowdown_ma_ma2);
avg_time_slowdown_rrt_ma2 = mean(time_slowdown_rrt_ma2);
% avg_time_speedup_dts_ma = mean(time_speedup_dts_ma);
% avg_time_speedup_ma_dts = mean(time_speedup_ma_dts);

%add in here
% avg_exp_speedup_dts_rr = mean(exp_speedup_dts_rr);
% avg_exp_speedup_ma_rr = mean(exp_speedup_ma_rr);
avg_exp_speedup_ma_ma2 = mean(exp_speedup_ma_ma2);
avg_exp_slowdown_ma_ma2 = mean(exp_slowdown_ma_ma2);
% avg_exp_speedup_rrt_ma2 = mean(exp_speedup_rrt_ma2);
% avg_exp_speedup_dts_ma = mean(exp_speedup_dts_ma);
% avg_exp_speedup_ma_dts = mean(exp_speedup_ma_dts);

fprintf('success  ma=%d  ma2=%d rrt=%d \n\n',success_ma, success_ma2, success_rrt);

% fprintf('rr/dts in_common=%d\n',in_common_dts_rr);
% fprintf('rr/dts speedups: time=%f expand=%f\n',avg_time_speedup_dts_rr,avg_exp_speedup_dts_rr);
% fprintf('rr/dts average time: dts=%f rr=%f\n', mean(all_times_dts(dts_rr)), mean(all_times_rr(dts_rr)));
% fprintf('rr/dts average exp: dts=%f rr=%f\n', mean(all_exp_dts(dts_rr)), mean(all_exp_rr(dts_rr)));
% fprintf('rr/dts path improvement: base=%f obj=%f arms=%f\n',...
%         mean(rr_paths.base(dts_rr)./dts_paths.base(dts_rr)),...
%         mean(rr_paths.obj(dts_rr)./dts_paths.obj(dts_rr)),...
%         mean(rr_paths.arm_sqr(dts_rr)./dts_paths.arm_sqr(dts_rr)));
% fprintf('rr/dts average path: dts_base=%f dts_obj=%f dts_arms=%f rr_base=%f rr_obj=%f rr_arms=%f\n\n', ...
%         mean(dts_paths.base(dts_rr)), mean(dts_paths.obj(dts_rr)), mean(dts_paths.arm_sqr(dts_rr)),...
%         mean(rr_paths.base(dts_rr)), mean(rr_paths.obj(dts_rr)), mean(rr_paths.arm_sqr(dts_rr)));

% fprintf('rr/ma in_common=%d\n',in_common_ma_rr);
% fprintf('rr/ma speedups: time=%f expand=%f\n',avg_time_speedup_ma_rr,avg_exp_speedup_ma_rr);
% fprintf('rr/ma average time: ma=%f rr=%f\n', mean(all_times_ma(ma_rr)), mean(all_times_rr(ma_rr)));
% fprintf('rr/ma average exp: ma=%f rr=%f\n', mean(all_exp_ma(ma_rr)), mean(all_exp_rr(ma_rr)));
% fprintf('rr/ma path improvement: base=%f obj=%f arms=%f\n',...
%         mean(rr_paths.base(ma_rr)./ma_paths.base(ma_rr)),...
%         mean(rr_paths.obj(ma_rr)./ma_paths.obj(ma_rr)),...
%         mean(rr_paths.arm_sqr(ma_rr)./ma_paths.arm_sqr(ma_rr)));
% fprintf('rr/ma average path: ma_base=%f ma_obj=%f ma_arms=%f rr_base=%f rr_obj=%f rr_arms=%f\n\n', ...
%         mean(ma_paths.base(ma_rr)), mean(ma_paths.obj(ma_rr)), mean(ma_paths.arm_sqr(ma_rr)),...
%         mean(rr_paths.base(ma_rr)), mean(rr_paths.obj(ma_rr)), mean(rr_paths.arm_sqr(ma_rr)));

% fprintf('rrt/dts in_common=%d\n',in_common_dts_rrt);
% fprintf('rrt/dts speedups: time=%f\n',avg_time_speedup_dts_rrt);
% fprintf('rrt/dts average time: dts=%f rrt=%f\n', mean(all_times_dts(dts_rrt)), mean(all_times_rrt(dts_rrt)));
% fprintf('rrt/dts path improvement: base=%f obj=%f arms=%f\n',...
%         mean(rrt_paths.base(dts_rrt)./dts_paths.base(dts_rrt)),...
%         mean(rrt_paths.obj(dts_rrt)./dts_paths.obj(dts_rrt)),...
%         mean(rrt_paths.arm_sqr(dts_rrt)./dts_paths.arm_sqr(dts_rrt)));
% fprintf('rrt/dts average path: dts_base=%f dts_obj=%f dts_arms=%f rrt_base=%f rrt_obj=%f rrt_arms=%f\n\n', ...
%         mean(dts_paths.base(dts_rrt)), mean(dts_paths.obj(dts_rrt)), mean(dts_paths.arm_sqr(dts_rrt)),...
%         mean(rrt_paths.base(dts_rrt)), mean(rrt_paths.obj(dts_rrt)), mean(rrt_paths.arm_sqr(dts_rrt)));

%---uncomment after adding paths 
% fprintf('rrt/ma in_common=%d\n',in_common_ma_rrt);
% fprintf('rrt/ma speedups: time=%f\n',avg_time_speedup_ma_rrt);
% fprintf('rrt/ma average time: ma=%f rrt=%f\n', mean(all_times_ma(ma_rrt)), mean(all_times_rrt(ma_rrt)));
% fprintf('rrt/ma path improvement: base=%f obj=%f arms=%f\n',...
%         mean(rrt_paths.base(ma_rrt)./ma_paths.base(ma_rrt)),...
%         mean(rrt_paths.obj(ma_rrt)./ma_paths.obj(ma_rrt)),...
%         mean(rrt_paths.arm_sqr(ma_rrt)./ma_paths.arm_sqr(ma_rrt)));
% fprintf('rrt/ma average path: ma_base=%f ma_obj=%f ma_arms=%f rrt_base=%f rrt_obj=%f rrt_arms=%f\n\n', ...
%         mean(ma_paths.base(ma_rrt)), mean(ma_paths.obj(ma_rrt)), mean(ma_paths.arm_sqr(ma_rrt)),...
%         mean(rrt_paths.base(ma_rrt)), mean(rrt_paths.obj(ma_rrt)), mean(rrt_paths.arm_sqr(ma_rrt)));
%------

%---uncomment after adding paths 
% fprintf('rrt/ma2 in_common=%d\n',in_common_ma2_rrt);
% fprintf('rrt/ma2 speedups: time=%f\n',avg_time_speedup_ma2_rrt);
% fprintf('rrt/ma2 average time: ma2=%f rrt=%f\n', mean(all_times_ma2(ma2_rrt)), mean(all_times_rrt(ma2_rrt)));
% fprintf('rrt/ma2 path improvement: base=%f obj=%f arms=%f\n',...
%         mean(rrt_paths.base(ma2_rrt)./ma2_paths.base(ma2_rrt)),...
%         mean(rrt_paths.obj(ma2_rrt)./ma2_paths.obj(ma2_rrt)),...
%         mean(rrt_paths.arm_sqr(ma2_rrt)./ma2_paths.arm_sqr(ma2_rrt)));
% fprintf('rrt/ma2 average path: ma2_base=%f ma2_obj=%f ma2_arms=%f rrt_base=%f rrt_obj=%f rrt_arms=%f\n\n', ...
%         mean(ma2_paths.base(ma2_rrt)), mean(ma2_paths.obj(ma2_rrt)), mean(ma2_paths.arm_sqr(ma2_rrt)),...
%         mean(rrt_paths.base(ma2_rrt)), mean(rrt_paths.obj(ma2_rrt)), mean(rrt_paths.arm_sqr(ma2_rrt)));
%------
fprintf('ma2/rrt in_common=%d\n',in_common_rrt_ma2);
fprintf('ma2/rrt speedups: time=%f \n',avg_time_speedup_rrt_ma2);
fprintf('ma2/rrt slowdowns: time=%f \n',avg_time_slowdown_rrt_ma2);
fprintf('ma2/rrt average time: rrt=%f ma2=%f\n', mean(all_times_rrt(rrt_ma2)), mean(all_times_ma2(rrt_ma2)));
% fprintf('ma2/rrt average exp: rrt=%f ma2=%f\n', mean(all_exp_rrt(rrt_ma2)), mean(all_exp_ma2(rrt_ma2)));
% fprintf('ma2/rrt path improvement: base=%f obj=%f arms=%f\n',...
%         mean(rrt_paths.base(rrt_ma2)./ma2_paths.base(rrt_ma2)),...
%         mean(rrt_paths.obj(rrt_ma2)./ma2_paths.obj(rrt_ma2)),...
%         mean(rrt_paths.arm_sqr(rrt_ma2)./ma2_paths.arm_sqr(rrt_ma2)));
% fprintf('ma2/rrt average path: rrt_base=%f rrt_obj=%f rrt_arms=%f ma2_base=%f ma2_obj=%f ma2_arms=%f\n\n', ...
%         mean(rrt_paths.base(rrt_ma2)), mean(rrt_paths.obj(rrt_ma2)), mean(rrt_paths.arm_sqr(rrt_ma2)),...
%         mean(ma2_paths.base(rrt_ma2)), mean(ma2_paths.obj(rrt_ma2)), mean(ma2_paths.arm_sqr(rrt_ma2)));
fprintf('ma2/rrt percentage speedup: %f\n',percentage_speedups_rrt)
fprintf('ma2/rrt percentage slowdowns: %f\n',percentage_slowdowns_rrt)
    
fprintf('ma2/ma in_common=%d\n',in_common_ma_ma2);
fprintf('ma2/ma speedups: time=%f expand=%f\n',avg_time_speedup_ma_ma2,avg_exp_speedup_ma_ma2);
fprintf('ma2/ma slowdowns: time=%f expand=%f\n',avg_time_slowdown_ma_ma2,avg_exp_slowdown_ma_ma2);
fprintf('ma2/ma average time: ma=%f ma2=%f\n', mean(all_times_ma(ma_ma2)), mean(all_times_ma2(ma_ma2)));
fprintf('ma2/ma average exp: ma=%f ma2=%f\n', mean(all_exp_ma(ma_ma2)), mean(all_exp_ma2(ma_ma2)));
% fprintf('ma2/ma path improvement: base=%f obj=%f arms=%f\n',...
%         mean(ma_paths.base(ma_ma2)./ma2_paths.base(ma_ma2)),...
%         mean(ma_paths.obj(ma_ma2)./ma2_paths.obj(ma_ma2)),...
%         mean(ma_paths.arm_sqr(ma_ma2)./ma2_paths.arm_sqr(ma_ma2)));
% fprintf('ma2/ma average path: ma_base=%f ma_obj=%f ma_arms=%f ma2_base=%f ma2_obj=%f ma2_arms=%f\n\n', ...
%         mean(ma_paths.base(ma_ma2)), mean(ma_paths.obj(ma_ma2)), mean(ma_paths.arm_sqr(ma_ma2)),...
%         mean(ma2_paths.base(ma_ma2)), mean(ma2_paths.obj(ma_ma2)), mean(ma2_paths.arm_sqr(ma_ma2)));
fprintf('ma2/ma percentage speedup: %f\n',percentage_speedups_ma)
fprintf('ma2/ma percentage slowdowns: %f\n',percentage_slowdowns_ma)
fprintf('ma2/ma percentage speedup exp: %f\n',percentage_speedups_exp_ma)
fprintf('ma2/ma percentage slowdowns exp: %f\n',percentage_slowdowns_exp_ma)
% fprintf('ma/dts in_common=%d\n',in_common_dts_ma);
% fprintf('ma/dts speedups: time=%f expand=%f\n',avg_time_speedup_dts_ma,avg_exp_speedup_dts_ma);
% fprintf('dts/ma speedups: time=%f expand=%f\n',avg_time_speedup_ma_dts,avg_exp_speedup_ma_dts);
% fprintf('ma/dts average time: dts=%f ma=%f\n', mean(all_times_dts(dts_ma)), mean(all_times_ma(dts_ma)));
% fprintf('ma/dts average exp: dts=%f ma=%f\n', mean(all_exp_dts(dts_ma)), mean(all_exp_ma(dts_ma)));
% fprintf('ma/dts average path: dts_base=%f dts_obj=%f dts_arms=%f ma_base=%f ma_obj=%f ma_arms=%f\n\n', ...
%         mean(dts_paths.base(dts_ma)), mean(dts_paths.obj(dts_ma)), mean(dts_paths.arm_sqr(dts_ma)),...
%         mean(ma_paths.base(dts_ma)), mean(ma_paths.obj(dts_ma)), mean(ma_paths.arm_sqr(dts_ma)));

fprintf('ma2 average time:  ma2=%f\n', mean(all_times_ma2(idx_ma2)));
fprintf('ma2 average exp:  ma2=%f\n', mean(all_exp_ma2(idx_ma2)));
% fprintf('ma2 average path: ma2_base=%f ma2_obj=%f ma2_arms=%f\n\n', ...
%         mean(ma2_paths.base(idx_ma2)), mean(ma2_paths.obj(idx_ma2)), mean(ma2_paths.arm_sqr(idx_ma2)));