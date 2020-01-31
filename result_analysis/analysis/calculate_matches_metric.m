function m = calculate_matches_metric (data)

m = (10*(data.avg_num_matches - data.avg_lost_keys)/data.avg_num_keys);
m = (10*(data.avg_num_matches)/data.avg_num_keys);


end
