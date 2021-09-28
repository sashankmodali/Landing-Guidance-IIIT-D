function ydot = thirdmodel(t,y,zto,kmax)
%     global vtarray
    global stopdist flagarray timearray psidotarray vpmax desarray sarray tfinal vpdotarr vparr rarray rxyarray rdotxyarray controlvector controlmax angledesired crossnum So tuningparam varyingconstants Barray detarray Ainvstored tstored Bstored initialvals
    persistent tfinalflag visflag anotherflag alphato psio Rzo psiflag vpo vto psidoto alphatdoto vpdotprev changeflag acc Rxyo Rdotxyo avgalphatdot avgalphat
    
    if isempty(tfinalflag)
        tfinalflag =0;
    end
    if isempty(visflag)
        visflag=0;
    end
    if isempty(anotherflag)
        anotherflag=1;
    end
    
    if isempty(vpdotprev)
        vpdotprev =0;
    end
    
    if isempty(changeflag)
        changeflag=1;
    end
    
    if isempty(psiflag)
        psiflag=1;
    end
    flag = 0;
    if ~flag
        varyingconstants=0;
    end
    alphat = y(1); vt = y(2); vp = y(3); gamma =y(4);alphap=y(5);xp=y(6);yp=y(7);zp=y(8);xt=y(9);yt=y(10);
    alphatused=wrapToPi(alphat);
%
%     for i=1:length(rov_traj)
%         if rov_traj(i,3)>=t
%             break
%         end
%     end
%     arg =i;
    Vw = 0;
    alphaw = pi/6;
    gammaw = pi/6;
    bounds =false;
%%
    Rxy = sqrt((xt-xp)^2 + (yt-yp)^2);
    if ~visflag && false
        Rxy = Rxy
    end
    Rz = zto - zp;
    R = sqrt(Rxy^2+Rz^2);
    psi = atan2(yt-yp,xt-xp);
    theta = atan2(-Rz,Rxy);
    if isempty(psio)
        psio=psi;
    end
    angled=psio;
    
    vtdot=0;
    if isempty(acc)
        acc = "highmaneuvering";
    end
%     alphatdot=0;
%     alphatdot=30*pi/180;
%     alphatdot=10*pi/180
    k=30*pi/180;
    alphatdot = k*sin(pi/2*t);
%     alphatddot=0;
    alphatddot= k*pi/2*cos(pi/2*t);
    Rdotxy = vt*cos(alphat - psi)-vp*cos(gamma)*cos(alphap-psi);
    if ~visflag && false
        Rdotxy = Rdotxy
    end
        
    if Rxy<7.5 && changeflag
      acc="normal";
      alphato=[];
      changeflag=0;
      So=[];
    end
   
    if isempty(alphato)
        alphato=alphat;
        Rzo=Rz;
        psio = psi;
        vpo = vp;
        vto = vt;
        Rxyo = Rxy;
        Rdotxyo = Rdotxy;
        psidoto = (vt*sin(alphato-psio) - vp*cos(gamma)*sin(alphap-psio))/Rxy;
        alphatdoto = alphatdot;
        initialvals(end+1,:)= [alphato*180/pi,Rzo,psio*180/pi,vpo,vto,Rxyo,Rdotxyo,psidoto*180/pi,alphatdoto*180/pi];
        if t~=0
            avgalphatdot = (alphatdot-initialvals(1,end))/t;
            avgalphat = (alphat-initialvals(1,1))/t;
        else
            avgalphatdot=alphatdoto;
            avgalphat = alphato;
        end
    end
    
    
    if Rxy>0.1 || R>0.1
        Rdotz = -vp*sin(gamma);
        psidot = (vt*sin(alphat-psi)-vp*cos(gamma)*sin(alphap-psi))/(Rxy);
        if psidot>800
            psiflag=1;
%             error("psidot is " + num2str(psidot)+"!");
        end
        thetadot = (vp*sin(gamma)*cos(theta)-Rdotxy*sin(theta))/R;
        xpdot = vp*cos(gamma)*cos(alphap);
        ypdot = vp*cos(gamma)*sin(alphap);
        zpdot = vp*sin(gamma);
        xtdot = vt*cos(alphat);
        ytdot = vt*sin(alphat);
        if abs(psidot)>2
            if anotherflag==1
                visflag =1;
                anotherflag=0;
%             anotherflag=1;
            end
        end
%         angledesired = 0;
        degreenum=5;
        degree = 7;
        if flag
            if Rxyo <= 40
                if alphaavg > 0.1
                    ka = abs(5*(log(1+vpo))^0.2*(vpo-vto)/(vpo+vto)/Rxyo^0.4);
                else
                    ka = abs(5*(log(1+vpo))^0.2*(vpo-vto)/(vpo+vto)/Rxyo^0.4);
                end
            elseif Rxyo <=60
                ka = abs(5*(log(1+vpo))^0.2*(vpo-vto)/(vpo+vto)/Rxyo^0.3);
            else
                ka = abs(5*(log(1+vpo))^0.2*(vpo-vto)/(vpo+vto)/Rxyo^0.1);
            end
        else
            if Rxyo <=10
                ka = abs(5*((vpo-vto)/(vpo+vto))/Rxyo^0.3);
            elseif Rxyo <= 40
                ka = abs(5*((vpo-vto)/(vpo+vto))/Rxyo^0.5);
            elseif Rxyo <=60
                ka = abs(5*((vpo-vto)/(vpo+vto))/Rxyo^0.6);
            else
                ka = abs(5*((vpo-vto)/(vpo+vto))/Rxyo^0.45);
            end
        end
        
        if ~flag
            kb = 2*ka;
            if acc=="highmaneuvering"
                kb=kb/2;
            end
        else
            kb = ((nthroot(wrapToPi(psio-(alphato+angledesired))^2/Rxyo^2,3)))*ka;
        end
        if varyingconstants
            kc = nthroot(tan(theta)^2,3)*ka;
        else
            kc = (((nthroot(Rzo^2/Rxyo^2,3)))^flag)*ka;
        end
%         k=kmax;
        if isempty(So)
            So = [Rdotxy + ka*Rxy*nthroot(Rxy^2,-3)^flag; Rdotz + kc*Rz*nthroot(Rz^2,-3)^flag; psidot - alphatdot + kb*(wrapToPi(psi-alphat- angledesired))*nthroot(wrapToPi(psi-alphat-angledesired)^2,-3)^flag];
            if acc=="highmaneuvering"
                So = [Rdotxy + ka*Rxy*nthroot(Rxy^2,-3)^flag; Rdotz + kc*Rz*nthroot(Rz^2,-3)^flag; psidot + kb*(wrapToPi(psi- angled))*nthroot(wrapToPi(psi-angled)^2,-3)^flag];
            end
            k1=3/2*nthroot(So(1)^(degree-degreenum),degree)*max([abs(Rdotxyo),ka*(Rxyo)])/Rxyo;
            k2=(nthroot(So(2)^(degree-degreenum)/So(1)^(degree-degreenum),degree)*k1);
            k3=(nthroot(So(3)^(degree-degreenum)/So(1)^(degree-degreenum),degree)*k1);
            tuningparam(end+1,:) = [ka,kb,k1,k2,k3];
        end
        k1=3/2*nthroot(So(1)^(degree-degreenum),degree)*max([abs(Rdotxyo),ka*(Rxyo)])/Rxyo;
        k2=(nthroot(So(2)^(degree-degreenum)/So(1)^(degree-degreenum),degree)*k1);
        k3=(nthroot(So(3)^(degree-degreenum)/So(1)^(degree-degreenum),degree)*k1);
        n=round(2*log(100)*k1/(3*nthroot(So(1)^2,3)*ka))+1;
%         if n>=8
%             k3=k3/7;
% %             k1 = k1/3;
% %             k2 = k2/3;
%         elseif n>=6
%             k3=k3/5;
% %             k1 = k1/2;
% %             k2 = k2/2;
%         elseif n>=4
%             k3=k3;
% %             k1 = k1/2;
% %             k2 = k2/2;
%         elseif n>=3
%             k3=k3;
% %             k1 = k1/2;
% %             k2 = k2/2;
%         end
% %         if abs(psio - (alphato +angledesired))>0.001
% %             k3 = 3/2*nthroot(So(3)^(degree-degreenum),degree)*max([abs(psidoto - alphatdoto),kb*nthroot(abs(psio - (alphato +angledesired)),3)])/abs(psio - (alphato +angledesired));
% %         end
        
%           k1=20;
%           k2=20;
%           k3=20;

%         S = [Rdotxy + k*Rxy; Rdotz - tan(theta)*Rxy; alphap - alphat - angledesired];
%         A = [vp*sin(alphap-psi)*cos(gamma), vp*cos(alphap-psi)*sin(gamma), -cos(alphap-psi)*cos(gamma); -tan(theta)*(vp*sin(alphap-psi)*cos(gamma)), -tan(theta)*(vp*cos(alphap-psi)*sin(gamma)), -tan(theta)*(-cos(alphap-psi)*cos(gamma))-sin(gamma);1, 0, 0];
%         B = [-k1*nthroot(S(1),3) - vtdot*cos(alphat-psi) + vp*sin(alphap-psi)*cos(gamma)+vt*sin(alphat-psi)*(alphatdot-psidot)+k*(vt*cos(alphat-psi)-vp*cos(alphap-psi)*cos(gamma)); - k2*nthroot(S(2),3)+tan(theta)*(vtdot*cos(alphat-psi)-vp*sin(alphap-psi)*cos(gamma)-vt*sin(alphat-psi)*(alphatdot-psidot));-k3*nthroot(S(3),3)+alphatdot];
        if flag
            S = [Rdotxy + ka*nthroot(Rxy,3); Rdotz + kc*nthroot(Rz,3); psidot - alphatdot + kb*nthroot(wrapToPi(psi-alphat-angledesired),3)];
            if acc=="highmaneuvering"
                S = [Rdotxy + ka*nthroot(Rxy,3); Rdotz + kc*nthroot(Rz,3); psidot + kb*nthroot(wrapToPi(psi-angled),3)];
            end
        else
            S = [Rdotxy + ka*Rxy; Rdotz + kc*Rz; psidot - alphatdot + kb*(wrapToPi(psi - alphat- angledesired))];
            if acc=="highmaneuvering"
                S = [Rdotxy + ka*Rxy; Rdotz + kc*Rz; psidot + kb*(wrapToPi(psi - angled))];
            end
        end
        
        A = [-cos(alphap-psi)*cos(gamma), vp*sin(alphap-psi)*cos(gamma), vp*cos(alphap-psi)*sin(gamma);-sin(gamma), 0, -vp*cos(gamma);-sin(alphap-psi)*cos(gamma), -vp*cos(alphap-psi)*cos(gamma), vp*sin(alphap-psi)*sin(gamma)];
        if abs(nthroot((wrapToPi(psi-(alphat+angledesired)))^2,3)) < 0.01 && flag
            angleterm=0;
        else
            angleterm = kb*Rxy*(psidot-alphatdot)*(1/3/nthroot((wrapToPi(psi-(alphat+angledesired)))^2,3))^flag;
            if acc=="highmaneuvering"
                angleterm = kb*Rxy*(psidot)*(1/3/nthroot((wrapToPi(psi-(angled)))^2,3))^flag;
            end
        end
        b1=-k1*nthroot(S(1,1)^degreenum,degree) + vp*sin(alphap-psi)*cos(gamma)*psidot - vtdot*cos(alphat-psi)+vt*sin(alphat-psi)*(alphatdot-psidot) - ka*(vt*cos(alphat-psi)-vp*cos(alphap-psi)*cos(gamma))*(1/3*nthroot(Rxy^2,-3))^flag;
        b2=-k2*nthroot(S(2,1)^degreenum,degree) - (kc*(Rdotz)*(1/3*nthroot(Rz^2,-3))^flag)-2/3/nthroot(tan(theta),3)*sec(theta)^2*thetadot*ka*nthroot(Rz,3)*varyingconstants;
        b3=-Rxy*k3*nthroot(S(3,1)^degreenum,degree) -angleterm + alphatddot*Rxy + Rdotxy*psidot - vp*cos(alphap-psi)*cos(gamma)*psidot - vt*cos(alphat-psi)*(alphatdot-psidot) - vtdot*sin(alphat-psi);
        if acc=="highmaneuvering"
            b3=-Rxy*k3*nthroot(S(3,1)^degreenum,degree) -angleterm + Rdotxy*psidot - vp*cos(alphap-psi)*cos(gamma)*psidot - vt*cos(alphat-psi)*(alphatdot-psidot) - vtdot*sin(alphat-psi);
        end
        
        B = [b1;b2;b3];
        i = length(timearray);
        while i > 0 && timearray(i)>t
            timearray(i)=[];
            flagarray(i)=[];
            psidotarray(i)=[];
            desarray(i)=[];
            sarray(:,i)=[];
            Barray(:,i)=[];
            detarray(i)=[];
            vparr(i)=[];
            vpdotarr(i)=[];
            rarray(i) = [];
            rxyarray(i) = [];
            rdotxyarray(i) = [];
            controlvector(:,i) =[];
            i=i-1;
        end

        if vp^2>0.01 && cos(gamma)>0.01 && vp>0%det(A) > 0.01
            C = inv(A)*B
            Rxy
            Rz
            if C(1) > 4.4 && tstored==0
                tstored = t;
                Ainvstored= inv(A);
                Bstored = B;
            end
%             Ainv = 1/vp^2/cos(gamma)^2*[-vp^2*cos(gamma)^2*cos(alphap-psi), -vp^2*cos(gamma)*sin(gamma), -vp^2*cos(gamma)^2*sin(alphap-psi); vp*sin(alphap-psi), 0, -vp*cos(alphap-psi); vp*cos(alphap-psi)*cos(gamma)*sin(gamma), -vp*cos(gamma)^2, vp*sin(alphap-psi)*cos(gamma)*sin(gamma)];
%             C = Ainv*B;
%             if C(1) > 10^3 || true
%                 psidot
%                 Rxy
%                 Rxypsidot = Rxy*psidot
%                 Rdotxy
%                 thedet = det(A)
%                 t
%                 S
% %                 error("blown up!");
%             end
            thedet = det(A);
            disp("psidot " + num2str(psidot))
            disp(thedet);
            disp("headspeed " + num2str([vp,gamma]))
            if thedet<0.001
                error("det is less than 0.001!!!")
            end
            if ~visflag && false
                thedet=thedet
            else
                if visflag
                    t
                    psidot
                    Rxy*psidot
                    Rxy
                    visflag=0;
                    if C(1) > 10^3
                        error("blown up!");
                    end
                end
            end
            alphapdot = C(2);
            gammadot = C(3); 
            vpdot = C(1);
            if abs(vpdotprev) >0.2 && abs(vpdot/vpdotprev) > 2
                vpdot= 2*sign(vpdot/vpdotprev)*vpdotprev;
            end
            if bounds
                if abs(vpdot)>5
                    vpdot = sign(vpdot)*5;
                end
                if abs(alphapdot) > pi/2
                    alphapdot = sign(alphapdot)*pi/2;
                end
                if abs(gammadot) > pi/2
                    gammadot = sign(gammadot)*pi/2;
                end
            end
            flagarray(end+1) = 1;
            timearray(end+1) = t;
            psidotarray(end+1) = psidot;
            desarray(end+1) = alphat - psi;
            sarray(:,end+1)=S;
            Barray(:,end+1) = B;
            detarray(end+1)=thedet;
            vpdotarr(end+1)=vpdot;
            vparr(end+1)=vp;
            rarray(end+1) = R;
            rxyarray(end+1) = Rxy;
            rdotxyarray(end+1) = Rdotxy;
            if length(C) == 3
                controlvector(:,end+1) = [vpdot;alphapdot;gammadot;acc];
            else
                error('wth is happening');
            end
        else
            alphapdot=0;
            gammadot=0;
            vpdot=0;
            if bounds
                if vp<=0.1
                    vpdot=1;
                end
                if cos(gamma)<0.01
                    gammadot=-sign(gamma)*pi/2;
                end
            end
            disp("this t" + num2str(t))
            disp("this vp " + num2str(vp))
            disp("this alphap" + num2str(alphap))
            disp("this gamma" + num2str(gamma))
            flagarray(end+1)= 0;
            timearray(end+1) = t;
            psidotarray(end+1) = psidot;
            desarray(end+1) = alphat - psi;
            sarray(:,end+1)=S;
            Barray(:,end+1) = B;
            detarray(end+1)=0;
            vpdotarr(end+1)=vpdot;
            vparr(end+1)=vp;
            rarray(end+1) = R;
            rxyarray(end+1) = Rxy;
            rdotxyarray(end+1) = Rdotxy;
            controlvector(:,end+1) = [0;0;0;acc];
            crossnum=crossnum+1;
        end
    else
        Rdotxy = 0;
        Rdotz =0;
        psidot = 0;
        gammadot=0;
        alphapdot = 0;
        thetadot=0;
%         Vrdot=0;
        xpdot=0;
        ypdot=0;
        xtdot=0;
        ytdot=0;
        zpdot=0;
        vpdot = 0;
        alphapdot=0;
        gammadot=0;
%         amdot=0;
        if ~tfinalflag
            tfinal=t;
            tfinalflag=1;
        end
        disp(['final time is ',num2str(tfinal)])
    end
    if vp>vpmax
        vpmax=vp;
    end
    vpdotprev=vpdot;
%     if vpdot> vpdotmax
%         vpdotmax=vpdot;
%     end
%     t
    ydot = [alphatdot;vtdot;vpdot;gammadot;alphapdot;xpdot;ypdot;zpdot;xtdot;ytdot];
end