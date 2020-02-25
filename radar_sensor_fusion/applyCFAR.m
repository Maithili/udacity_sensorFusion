function cfar_thresh = applyCFAR(config, offset, matrix, cd, cr)
    cfar_thresh = 0;
    num_indices = length(config.doppler.indexoffset);
    for i = 1:num_indices
        cfar_thresh = cfar_thresh + db2pow(matrix(cr+config.range.indexoffset(i),cd+config.doppler.indexoffset(i)));
    end
    cfar_thresh = cfar_thresh/num_indices;
    cfar_thresh = pow2db(cfar_thresh);
    cfar_thresh = cfar_thresh + offset;
end