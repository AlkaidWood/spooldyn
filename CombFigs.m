function hf = CombFigs(varargin)
% CombFigs（varargin）将不同figures图片合并到一个figure中
% 调用格式 
% 极简方式： 
%          CombFigs()  不带任何输入参数，将当前目录下所有fig文件合并成一个fig文件
%                      默认合并后的fig文件名为  合并Figure文件.fig
% 指定目标fig文件名：
%          CombFigs(figname)  figname为合并后的fig文件名，将当前目录下所有fig文件合并为figname文件 
% 指定目标fig文件名和要合并的文件名称
%          CombFigs(figname，figfiles)  figname为合并后的fig文件名,
%                                       figfiles为待合并的fig文件，cell数组格式，将figfiles文件合并为figname文件 
% By ZFS@wust 20210905

if  isempty(varargin)                   % 不输入
    files = dir( '*.fig' );
    filenames = { files(:).name };
    filenames0 = filenames{1};        % 合并目标文件
    figname = '合并Figure文件.fig';    % 默认保存文件名
elseif length(varargin) == 1          % 输入一个元素时，为复制目标文件名
    files = dir('*.fig');
    filenames = {files(:).name};
    filenames0 = filenames{1};     % 合并目标文件
    figname = varargin{1};         % 保存文件名
else                               % 输入多个元素时，每个元素为fig文件名
    figname = varargin{1};       % 第1个参数：保存文件名
    filenames  = varargin{2};      % 第2个参数：待合并的fig文件列表，字符串cell数组
    filenames0 = filenames{1};     % 合并目标文件
end

if isempty(figname)                  % 当filenames2输入为空占位符时
    figname = '合并Figure文件.fig';   % 默认保存文件名
end

if isempty(filenames)
    error('没有可供合并的figures文件')
else
    hf = open(filenames0);
    ax = findall(hf,'type','axes');      % axes句柄
    hg = findall(hf,'type','legend');   % legend句柄
    if isempty(hg)
        for ii=1:length(allchild(ax))
            hS(ii)={[filenames0  '曲线' num2str(ii)]};
        end
    else
        hS=hg.String;
    end
end

for ii = 1:length( filenames )
    if strcmp(filenames{ii},filenames0)
        continue;
    end
    h2= openfig(filenames{ii}, 'invisible');
    ax2 = findall(h2,'type','axes');
    hL = allchild(ax2);
    copyobj(hL,ax);   % 复制曲线
    hg2 = findall(h2,'type','legend');
    if isempty(hg2)
        for kk = 1:length(hL)
            jj = length(hS);
            hS(jj+1) = { [filenames{ii} '曲线' num2str(kk)]};
        end
    else
        jj = length(hS);
        n = length(hg2.String);
        hS(jj+1:jj+n) = hg2. String;
    end
end
%legend(ax,hS);
legend off
%savefig(hf,figname)          % 保存figure文件
% 删除光源，仅保留前两个
allLights = findobj(ax, 'Type', 'light');
if length(allLights) > 1
    delete(allLights(3:end)); % 保留前两个光源，删除其余
end

end


