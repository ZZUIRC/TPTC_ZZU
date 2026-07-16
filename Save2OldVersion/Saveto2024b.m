mdl = bdroot;

% 先保存当前模型
save_system(mdl);

% 导出文件名
outFile = fullfile(pwd, [mdl '_R2024b.slx']);

try
    Simulink.exportToVersion( ...
        mdl, ...
        outFile, ...
        'R2024b', ...
        AllowPrompt=true);
catch ME
    fprintf('\n========== 完整错误信息 ==========\n');
    fprintf('%s\n', getReport(ME, 'extended', 'hyperlinks', 'off'));
end