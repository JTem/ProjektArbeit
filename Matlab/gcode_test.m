%% Gcode reader
% ein kleines tool zum extrahieren der Kartesischen koordinaten aus gcode
% dateien, um sie in .csv datein abzuspeichern

clf;
clear all;
close all;

%öffnet gcode
raw_gcode_file = fopen('zahnrad.gcode');

%variablen initialisieren
j=0;
start = 0;
z_total = 0.1;

%Schleife die durchlaufen wird bis man ans ende der datei kommt
while ~feof(raw_gcode_file)
    
    %actuelle zeile der datei
    file_line = fgetl(raw_gcode_file);
    
    % zeile wird in einträge unterteilt Bsp.: 'G1' 'X123.111' 'Y122.001'...
    splitLine = strsplit(file_line);
    
    %wird für jeden eintrag in einer zeile einmal durchlaufen
    for i = 1:length(splitLine)

        %sucht nach T0 um die aufzeichnung der datenpunkte anzufangen,
        %davor stehen viele infos die man nicht braucht
        if strcmp(splitLine{i}, 'T0')
            start = 1;
            z_total = 0.1;
        end
        %infos nach end werden nichtmehr benötigt. hört hier auf mit der
        %aufzeichnung
        if strcmp(splitLine{i}, 'end')
            start = 0;
        end
        
        %aufzeichnung beginnt wenn start = true = 1
        if start
            %j=j+1;
            % sucht in aktueller zeile nach eintrag 'Z' und speichter was
            % hinter z folgt als z_total. z wird nur geändert wenn ein
            % neuer layer beginnt
            if splitLine{i}(1)=='Z'
                z_total = str2double(splitLine{i}(2:end));
            end
            %sucht in Zeile nach G1, danach folgen Kart. Koordinaten
            if strcmp(splitLine{i}, 'G1')
                %Koordinaten werden wie bei Z ausgelesen, danach auch
                %direkt in der datenMatrix gespeichert
                if splitLine{i+1}(1) == 'X'
                    j=j+1;
                    dataM(j,1) = str2double(splitLine{i+1}(2:end));
                    dataM(j,3) = z_total;
                    dataM(j,4) = 0;
                    dataM(j,5) = 0;
                    dataM(j,6) = 1;
                end
                if splitLine{i+2}(1)=='Y'
                    dataM(j,2) = str2double(splitLine{i+2}(2:end));
                end
            end
        end
    end
end
%csv datei wird erzeugz
csvwrite('Benchy.csv', dataM);