function N = Nsim_requerido(zeta_r)
% NSIM_REQUERIDO Calcula el número mínimo de simulaciones
% necesarias para obtener un error relativo zeta_r.
%
% Entrada:
%   zeta_r - Error relativo deseado (por ejemplo, 0.1 para 10%)
%
% Salida:
%   N - Número mínimo de simulaciones requerido

   if zeta_r <= 0 || zeta_r >= 1
        error('zeta_r debe estar entre 0 y 1 (ej: 0.1 para 10%%)');
    end

    N = ceil(2 / zeta_r^2);  % ✅ esta es la fórmula correcta
end