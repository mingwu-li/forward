function prob = ode_construct_forward(prob, tbid, data, sol)
%ODE_CONSTRUCT_FORWARD   Append an instance of 'forward' to problem.
%
% Add collocation and continuity conditions, monitor functions that
% evaluate to the problem parameters, and corresponding inactive
% continuation parameters.
%
% PROB = ODE_CONSTRUCT_FORWARD(PROB, TBID, DATA, SOL)
%
% PROB - Continuation problem structure.
% TBID - Toolbox instance identifier.
% DATA - Toolbox data structure.
% SOL  - Initial solution guess.

prob = coco_add_func(prob, tbid, @forward, data, 'zero', ...
  'u0', sol.u, 'F+dF', 't0', sol.t0);

if ~isempty(data.pnames)
  pfid = coco_get_id(tbid, 'pars');
  uidx = coco_get_func_data(prob, tbid, 'uidx'); % Context-dependent index array
  prob = coco_add_pars(prob, pfid, uidx(data.p_idx), data.pnames);
end

if data.autonomous
  t0fid = coco_get_id(tbid, 'T0');
  uidx  = coco_get_func_data(prob, tbid, 'uidx'); % Context-dependent index array
  prob  = coco_add_pars(prob, t0fid, uidx(data.T0_idx), t0fid);
end

prob = coco_add_slot(prob, tbid, @coco_save_data, data, 'save_full');

end

function [data, y, J] = forward(prob, data, u) %#ok<INUSL>
x0 = u(data.x0_idx);
x1 = u(data.x1_idx);
T0 = u(data.T0_idx);
T  = u(data.T_idx);
p  = u(data.p_idx);

if data.MATLABsolver
    if nargout<3
      if data.autonomous
        f = @(t,x) data.f(x, p);
      else
        f = @(t,x) data.f(t, x, p);
      end

      if data.par_opts.parallel
        subdim = data.par_opts.subdim;
        nsegs  = numel(x0)/subdim;
        x0mat  = reshape(x0,[subdim, nsegs]);
        x1mat  = zeros(subdim,nsegs);
        odesol = data.ODEsolver;
        odeopt = data.ode_opts;
        parfor i=1:nsegs
            [~, x] = odesol(f, [T0 T0+T], x0mat(:,i), odeopt);
            x1mat(:,i) = x(end,:)';
        end
        y = x1mat(:)-x1;
        
      else
        [~, x] = data.ODEsolver(f, [T0 T0+T], x0, data.ode_opts);
        y = x(end,:)' - x1;
      end
      
    else
      m  = data.dim;
      n  = data.pdim;
      
      if data.par_opts.parallel
          subdim = data.par_opts.subdim;
          nsegs  = m/subdim;
          x0mat  = reshape(x0,[subdim, nsegs]);
          x1mat  = zeros(subdim,nsegs);
          Jxmat  = zeros(subdim,subdim,nsegs);
          JT0mat = zeros(subdim,nsegs);
          JTmat  = zeros(subdim,nsegs);
          Jpmat  = zeros(subdim,n,nsegs);
          parfor i=1:nsegs
              if data.autonomous
                  f0 = data.f(x0mat(:,i), p);
              else
                  f0 = data.f(T0, x0mat(:,i), p);
              end
              M0 = [x0mat(:,i) eye(subdim,subdim) -f0 zeros(subdim,n)]; % x, \partial_x0, \partial_T0, \partial_p
              f  = @(t,M) VarEQN(data, subdim, n, t, M, p);

              [~, x] = data.ODEsolver(f, [T0 T0+T], M0(:), data.ode_opts);

              M1 = x(end,:);
              x1mat(:,i) = M1(1:subdim)';
%               y  = M1(1:m)' - x1;
              Jxmat(:,:,i) = reshape(M1(subdim+1:subdim+subdim*subdim), subdim, subdim);
              JT0mat(:,i) = M1(subdim+subdim*subdim+1:2*subdim+subdim*subdim)';
              if data.autonomous
                JTmat(:,i) = data.f(M1(1:subdim)', p);
              else
                JTmat(:,i) = data.f(T0+T, M1(1:subdim)', p);
              end
              JT0mat(:,i) = JT0mat(:,i) + JTmat(:,i);
              Jpmat(:,:,i)  = reshape(M1(2*subdim+subdim*subdim+1:end), subdim, n);     
          end
          y = x1mat(:)-x1;
          Jx = zeros(m,m);
          Jp = zeros(m,n);
          for i=1:nsegs
              idx = (i-1)*subdim+1:i*subdim;
              Jx(idx,idx) = Jxmat(:,:,i);
              Jp(idx,:) = Jpmat(:,:,i);
          end
          J = [ Jx -eye(m,m) JT0mat(:) JTmat(:) Jp ];
                   
      else
          if data.autonomous
              f0 = data.f(x0, p);
          else
              f0 = data.f(T0, x0, p);
          end
          M0 = [x0 eye(m,m) -f0 zeros(m,n)]; % x, \partial_x0, \partial_T0, \partial_p
          f  = @(t,M) VarEQN(data, m, n, t, M, p);

          [~, x] = data.ODEsolver(f, [T0 T0+T], M0(:), data.ode_opts);

          M1 = x(end,:);
          y  = M1(1:m)' - x1;
          Jx = reshape(M1(m+1:m+m*m), m, m);
          JT0 = M1(m+m*m+1:2*m+m*m)';
          if data.autonomous
            JT  = data.f(M1(1:m)', p);
          else
            JT  = data.f(T0+T, M1(1:m)', p);
          end
          JT0 = JT0 + JT;
          Jp  = reshape(M1(2*m+m*m+1:end), m, n);
          J   = [ Jx -eye(m,m) JT0 JT Jp ];
      end
    end
else
    switch data.order
        case 1
            [xend,Js] = data.ODEsolver(data.f, data.fx, data.fp, T0, T, ...
                x0, p, data.autonomous, data.ode_opts);
            
            y = xend-x1;
            if data.autonomous
                JT = data.f(xend,p);
            else
                JT = data.f(T0+T,xend,p);
            end
            JT0 = Js(:,1)+JT;
            Jx0 = Js(:,2:data.dim+1);
            Jp  = Js(:,data.dim+2:end);
            J   = [Jx0 -eye(data.dim) JT0 JT Jp];
            
        case 2
            if nargout<3
                [uend,vend] = data.ODEsolver(data.M, data.N, data.Nu, data.Nv, ...
                    data.Np, data.F, data.Fp, T0, T, x0, p, data.ode_opts);
                % rewrite results in first-order form
                y   = [uend;vend]-x1;                
            else
                [uend,vend,Jup,Jvp] = data.ODEsolver(data.M, data.N, data.Nu, data.Nv, ...
                    data.Np, data.F, data.Fp, T0, T, x0, p, data.ode_opts, 'var');
                % rewrite results in first-order form
                y   = [uend;vend]-x1;
                if ~isempty(data.Np)
                    Nend = data.N(uend,vend,p); % _p stands for evaluation at the previous step
                else
                    Nend = data.N(uend,vend);
                end
                if ~isempty(data.Fp)
                    Fend = data.F(T0+T,p);    % _0 stands for evaluation at the initial time
                else
                    Fend = data.F(T0+T);
                end
                JT  = [vend; data.M\(Fend-Nend)];
                JT0 = [Jup(:,1); Jvp(:,1)]+JT;
                Jx0 = [Jup(:,2:data.dim+1); Jvp(:,2:data.dim+1)];
                Jp  = [Jup(:,data.dim+2:end); Jvp(:,data.dim+2:end)];
                J   = [Jx0 -eye(data.dim) JT0 JT Jp];
            end
    end
end
end

function y = VarEQN(data, m, n, t, xx, p)
M  = reshape(xx(m+1:m+m*m), m, m);
L  = xx(m+m*m+1:2*m+m*m);
N  = reshape(xx(2*m+m*m+1:end), m, n);
if data.autonomous
  Fx = data.fx(xx(1:m), p);
  Fp = data.fp(xx(1:m), p);
  y  = [ data.f(xx(1:m), p) Fx*M Fx*L Fx*N+Fp ];
else
  Fx = data.fx(t, xx(1:m), p);
  Fp = data.fp(t, xx(1:m), p);
  y  = [ data.f(t, xx(1:m), p) Fx*M Fx*L Fx*N+Fp ];
end
y  = y(:);
end
