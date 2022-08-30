import re
template = 'tools/tmpl_FlightMode.m';
dest = "tools/FlightMode_.m";

variables = [];

with open(template, 'r') as tmpl, open(dest, 'w') as dest:
    tag_re = re.compile(r'^(.*?)%gen:\s+(?:(.*?)(?:\:\s*)?)*');
    for line in tmpl.readlines():
        line_dest = line
        res = tag_re.findall(line);
        if len(res) > 0:
            res = res[0];
            pref = res[0]
            commands = res[1:]
            if commands[0] == "variables":
                variables = commands[1].split(' ')
                line_dest = "";
            elif commands[0] == "setters":
                pattern = """
                function mode = set{}(val)
                    mode.mode_.{} = val
                end
                """
                line_dest = '\n'.join([
                    pattern.format(varName.title(), varName) for varName in variables
                ])

            print(res)
        print(line_dest, file=dest)
