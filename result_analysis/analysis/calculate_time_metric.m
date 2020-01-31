function m = calculate_time_metric (data)

m = (10/(data.avg_time_descriptor + data.avg_time_detector)^(1/3));

end