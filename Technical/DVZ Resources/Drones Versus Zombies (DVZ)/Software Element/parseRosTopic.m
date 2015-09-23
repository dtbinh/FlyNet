function a = parseRosTopic(file)
f = fileread(file);
f = strsplit(f, '\n');

a = struct();

c = 1;
for i = 1 : length(f)
	try
		disp([num2str(i), '/', num2str(length(f))])
		str = strtrim(f{i});
		if strcmp(str, '---')
			c = c + 1;
			continue
		end
		if str(end)==':'
			continue
		end
		field = strtrim(str(1:strfind(str, ':')-1));
		value = strtrim(str(strfind(str, ':')+1:end));

		if ~isfield(a, field)
			a.(field) = {};
		end
		b = a.(field);
		b{end+1} = value;
		a.(field) = b;			
	catch
	end
end
names = fieldnames(a);
for i = 1 : length(names)
	b = a.(names{i});
	b = str2double(b);
	if sum(isnan(b)) < length(b)/2
		a.(names{i}) = b;

	end
end