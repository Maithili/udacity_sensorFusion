load('final.mat');

detectors = [""];
for (i=1:30)
    if (~any(find(detectors == data(i).detector)))
        detectors = [detectors, data(i).detector];
    end
end
detectors(1) = [];

descriptors = [""];
for (i=1:30)
    if (~any(find(descriptors == data(i).descriptor)))
        descriptors = [descriptors, data(i).descriptor];
    end
end
descriptors(1) = [];

[det_grid, desc_grid] = meshgrid(1:length(detectors), 1:length(descriptors));

metrics = zeros(length(detectors), length(descriptors));
metrics_flat = [0,0,0];

for(i = 1:30)
    d1 = find(detectors==data(i).detector,1);
    d2 = find(descriptors==data(i).descriptor,1);
    
    m = calculate_matches_metric(data(i));
    metrics(d1,d2) = m;
    mt = calculate_time_metric(data(i));
    metrict(d1,d2) = mt;
    
    metric_combine(d1,d2) = mt+m;
    
    new_data = [d1, d2, m];
    metrics_flat = [metrics_flat;new_data];
end

figure;
disp ("Matches metric");
disp (metrics);
surf(det_grid, desc_grid, metrics);
title("matches metric");

figure;
disp ("Time metric");
disp (metrict);
surf(det_grid, desc_grid, metrict);
title("time metric");

figure;
disp ("Combined metric");
disp (metric_combine);
surf(det_grid, desc_grid, metric_combine);
title("sum of metrics");