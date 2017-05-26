close all
clc
clear all
warning off

% Create the webcam object.
cam = webcam();
frame = snapshot(cam);
%% parametros do algoritmo de rastreamento

% Get images size.
width = size(frame,2);
height = size(frame,1);

videoPlayer = vision.VideoPlayer('Position',[200 100 width+10 height+10]);
% videoPlayer2 = vision.VideoPlayer('Position',[300+width 100 width+10 height+10]);

%define tamanho do rastro
n=70;

%define quantidade minima de pixels vermelhos
area_min = width*height*0.3;

%define medicao inicial
ym=1;
xm=1;
Yc(1)=height/2;
Xc(1)=width/2;

%janela de interesse
jn = 0.1;

%% parametros do filtro de Kalman
Ts = 0.5 %tempo de amostragem

%matriz de estados para movimento 2D
A= [1 0 Ts 0;...
    0 1 0 Ts;...
    0 0 1 0;...
    0 0 0 1]
%matriz de entrada para MRUV
B=[0.5*Ts*Ts 0;...
    0 0.5*Ts*Ts;...
    Ts 0;...
    0 Ts]

%matriz de medicao
H=[1 0;... %observa x
    0 1;... %observa y
    0 0;... %nao observa dx
    0 0]' %nao observa dy

%matriz de covariancia de estado inicial
P=eye(4)*10e+15;

%matriz de covariancia dos estados (precisao do modelo)
Q=eye(4)*10e-1;

%matriz de covariance do sensor (qualidade do sensor)
R=eye(2)*10e-3

%define estado inicial
X(:,1,1)=[width/2 height/2 0 0]'; %centro parado

%define ganho inicial
K=zeros(4,2);

%matriz de covariancia de estado inicial
P2=eye(4)*10e+5;

%matriz de covariancia dos estados (precisao do modelo)
Q2=eye(4)*10e-6;

%matriz de covariance do sensor (qualidade do sensor)
R2=eye(2)*10e-3;

%define estado inicial
X2(:,1,1)=[width/2 height/2 0 0]'; %centro parado

%define ganho inicial
K2=zeros(4,2);

%define entrada inicial
u = zeros(2,1);
area_media(1)=area_min;
%%

v1 = uint8(1:width);
for i=1:height
    A00(i,:)=v1;
end
v2=uint8(1:height);
for i=1:width
    B00(:,i)=v2';
end

N=5000;
%figure('units','normalized','outerposition',[0 0 1 1])

for k=1:N
    k=k+1;
    I0=frame;
    %I0(:,400:550,1)=0;
    %% algoritmo de rastreamento
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %limiarizacao
    %separa pixels vermelhos
    L = I0(:,:,1)-I0(:,:,2)-I0(:,:,3);
    
    %faz limiarizacao
    L = L>100;
    I=im2uint8(L);
    
    %calcula area dos pixels vermelho
    area = sum(sum(I));
    crop = 0;
    
    %% considera area media do objeto
    if area > area_min
        area_media = (area+area_media)/2;
        area_min = 0.5*area_media;
        if k>30
            if abs(area-area_media) > 0.5*area_media
                area = 0;
            end
        end
    end
    %% pega posicao do objeto
    if area > area_min
        flag=0;
        %crop regiao de interesse
        J = sum(I); %projecao em x
        [m xm] = max(J);
        J = sum(I'); %projecao em y
        [m ym] = max(J);
        Yc(k)=ym;
        Xc(k)=xm;
        if (ym > jn*height+1) && (xm > jn*width+1)
            if (ym < (1-jn)*height) && (xm < (1-jn)*width)
                I = I(ym-jn*height:ym+jn*height,xm-jn*width:xm+jn*width);
                A0 = A00(ym-jn*height:ym+jn*height,xm-jn*width:xm+jn*width);
                B0 = B00(ym-jn*height:ym+jn*height,xm-jn*width:xm+jn*width);
                
                %calcula centro geometrico dos pixels
                %Xc(k) = xm;
                %Yc(k) = ym;
                Xc(k) = mean(round(sum(sum(I.*A0).*[1:size(I,2)])/area) + xm-jn*width,xm);
                Yc(k) = mean(round(sum(sum((I.*B0)').*[1:size(I,1)])/area) + ym-jn*height,ym);
                crop = 1;
            end
        end
        
        if crop == 0
            %calcula centro geometrico dos pixels
            
            Xc(k) = round(sum(sum(I.*A00).*[1:size(I,2)])/area);
            Yc(k) = round(sum(sum((I.*B00)').*[1:size(I,1)])/area);
            ym = Yc(k);
            xm = Xc(k);
        end
    else
        Xc(k)=X(1,1,1);
        Yc(k)=X(2,1,1);
    end
    
    %hold medicao
    if (Xc(k) < 1) || (Xc(k) > width)
        Xc(k)=Xc(k-1);
    end
    if (Yc(k) < 1) || (Yc(k) > height)
        Yc(k)=Yc(k-1);
    end
    
    %% %%%%%%%%%%%%%% KALMAN FILTER 1%%%%%%%%%%%%%%%%%
    %%%%% confia na medicao
    %State estimation
    X(:,1,k) = A*X(:,1,k-1)+B*u; %state prediction
    
    if area > area_min
        %estima entrada atual
        ax = X(3,1,k)-X(3,1,k-1);
        ay = X(4,1,k)-X(4,1,k-1);
        u = [ax ay]'/Ts;
    end
    
    %medicao
    z(:,1,k) = [Xc(k) Yc(k)]';
    
    %Estimative
    Zest = H*X(:,1,k); %measurement prediction
    
    if area > area_min
        v = z(:,1,k)-Zest; %measurement residual
        X(:,1,k) = X(:,1,k) + K*v; %update state estimate
    end
    
    %State covariance estimation
    P = A*P*A' + Q; %state prediction covariance
    S = H*P*H'+R; %measurement prediction covariance
    K = P*H'*inv(S); %filter gain
    P = (eye(4)-K*H)*P*(eye(4)-K*H)'+K*R*K'; %update covariance matrix
    
    %% %%%%%%%%%%%%% KALMAN FILTER 2%%%%%%%%%%%%%%%%%
    %%%% confia na estimativa
    
    %State estimation
    X2(:,1,k) = A*X2(:,1,k-1)+B*u; %state prediction
    
    %Estimative
    Zest = H*X2(:,1,k); %measurement prediction
    
    if area > area_min
        v = z(:,1,k)-Zest; %measurement residual
        X2(:,1,k) = X2(:,1,k) + K2*v; %update state estimate
    end
    
    %State covariance estimation
    P2 = A*P2*A' + Q2; %state prediction covariance
    S = H*P2*H'+R2; %measurement prediction covariance
    K2 = P2*H'*inv(S); %filter gain
    P2 = (eye(4)-K2*H)*P2*(eye(4)-K2*H)'+K2*R2*K2'; %update covariance matrix
    
    %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %Saída do filtro de Kalman
    if area > area_min
        Xc0(k)=X(1,1,k); %posicao eixo x
        Yc0(k)=X(2,1,k); %posicao eixo y
    else
        Xc0(k)=X2(1,1,k); %posicao eixo x
        Yc0(k)=X2(2,1,k); %posicao eixo y
    end
    
    %%% mostra medicao
    if area>area_min
        I0 = insertObjectAnnotation(I0,'rectangle',[Xc(k)-n/2 Yc(k)-n/2 n n],'medida');
    end
    
    if (Xc0(k) < 1) || (Xc0(k) > width)
        Xc0(k)=Xc0(k-1);
    end
    if (Yc0(k) < 1) || (Yc0(k) > height)
        Yc0(k)=Yc0(k-1);
    end
    
    %resultado filtrado
    I0 = insertObjectAnnotation(I0,'rectangle',[Xc0(k)-n/2 Yc0(k)-n/2 n n],'kalman','Color','green');
    if k>30
        I0 = insertMarker(I0,[Xc0(end-30:end)' Yc0(end-30:end)'],'x','Color','yellow');
    else
        I0 = insertMarker(I0,[Xc0(1:end)' Yc0(1:end)'],'x','Color','yellow');
    end
    if crop==1
        L(:,xm)=1;
        L(ym,:)=1;
        L(ym-jn*height:ym+jn*height,xm-jn*width)=1;
        L(ym-jn*height:ym+jn*height,xm+jn*width)=1;
        L(ym-jn*height,xm-jn*width:xm+jn*width)=1;
        L(ym+jn*height,xm-jn*width:xm+jn*width)=1;
    end
    
    frame = double(L);
    clear A0 B0
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    step(videoPlayer,I0);
    %step(videoPlayer2,frame);
    %% real time
    %subplot(1,2,1)
    %imshow(I0)
    %     if area>area_min
    %         title('Filtrando...')
    %     else
    %         title('Estimando...')
    %     end
    %hold on
    %% limiarizacao
    %subplot(1,2,2)
    %imshow(frame)
    %title(['frame: ' int2str(k-1)])
    
    %% get next frame
    frame = snapshot(cam);
end
release(videoPlayer);
clear cam

