function [results, data_calibrated] = calibrateImu(data_synced, options, import_results)
%calibrateImu [DEPRECATED]
if ~exist('import_results','var')
    import_results = struct();
end
[results, data_calibrated] = imuCalibrate(data_synced, options, import_results)