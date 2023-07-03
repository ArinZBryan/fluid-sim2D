using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FluidSimFunctions
{
    static void swap<T>(ref T a, ref T b) { T t = a; a = b; b = t; }
    public class SolveState
    {
        public int N;
        public float dt, diff, visc;
        public float[,] velocityX;
        public float[,] velocityY;
        public float[,] velocity_prevX;
        public float[,] velocity_prevY;
        public float[,] density;
        public float[,] density_prev;

        public SolveState(int N, float deltaTime, float diffusionRate, float viscosity)
        {
            this.N = N;
            this.dt = deltaTime;
            this.diff = diffusionRate;
            this.visc = viscosity;

            this.velocityX = new float[N + 2, N + 2];
            this.velocityY = new float[N + 2, N + 2];
            this.velocity_prevX = new float[N + 2, N + 2];
            this.velocity_prevY = new float[N + 2, N + 2];
            this.density = new float[N + 2, N + 2];
            this.density_prev = new float[N + 2, N + 2];
        }


    }
    public class Solver
    {
        SolveState currentState;
        enum boundary { IGNORE, HORIZONTAL, VERTICAL }

        public Solver(int N, float deltaTime, float diffusionRate, float viscosity)
        {
            currentState = new SolveState(N, deltaTime, diffusionRate, viscosity);
        }

        //Add arbritrary source array to arbritrary destination array
        float[,] add_source(int N, float[,] dest, float[,] sources, float dt)
        {
            for (int i = 0; i < N + 2; i++)
                for (int j = 0; j < N + 2; j++)
                    dest[i, j] += dt * sources[i, j];
            return dest;
        }

        //Set boundary conditions for arbritrary array x
        float[,] set_bnd(int N, boundary boundaryType, float[,] x)
        {
            for (int i = 1; i <= N; i++)    //Performs boundary checks for all edges of the array
            {
                //NOTE: the boundaryType enum is used to ensure that boundary conditions for a particular direction are only applied during the right passes
                //		This is to prevent the corners from being erroneously dealt with twice
                x[0, i] = (boundaryType == boundary.HORIZONTAL) ? -x[1, i] : x[1, i];       //left edge
                x[N + 1, i] = (boundaryType == boundary.HORIZONTAL) ? -x[N, i] : x[N, i];       //right edge
                x[i, 0] = (boundaryType == boundary.VERTICAL) ? -x[i, 1] : x[i, 1];         //top edge
                x[i, N + 1] = (boundaryType == boundary.VERTICAL) ? -x[i, N] : x[i, N];     //bottom edge
            }

            // Diffusion into corner cells
            // This is done now because the corner cells are only partially updated during the boundary checks above
            x[0, 0] = 0.5f * (x[1, 0] + x[0, 1]);
            x[0, N + 1] = 0.5f * (x[1, N + 1] + x[0, N]);
            x[N + 1, 0] = 0.5f * (x[N, 0] + x[N + 1, 1]);
            x[N + 1, N + 1] = 0.5f * (x[N, N + 1] + x[N + 1, N]);

            return x;
        }

        //This is the meat of the diffusion function, and uses a method like the Gauss-Seidel relaxation method to diffuse the array stably
        float[,] lin_solve(int N, boundary boundaryType, float[,] x, float[,] x0, float a, float c)
        {
            for (int k = 0; k < 20; k++)    //20 iterations
            {
                //For each cell in the array, set the value to the average of the surrounding cells
                for (int i = 1; i <= N; i++)
                    for (int j = 1; j <= N; j++)

                        //For each cell in the array, set the value to be the sum of the surrounding cells, times the diffusion rate a
                        //This method is derived from the 'diffuse_bad' function in the original stable fluids paper, but replacing the final -4x_n(i,j) with -4x_n+1(i,j)
                        //This allows for the method to be made more stable by using a more implicit method, which is common in fluid simulation
                        //The c term is just a remnant of the rearrangement, and comes to the RHS, when solving for x_n+1(i,j)

                        x[i, j] = (x0[i, j] + a * (x[i - 1, j] + x[i + 1, j] + x[i, j - 1] + x[i, j + 1])) / c;
                x = set_bnd(N, boundaryType, x);
            }
            return x;
        }

        //Diffuse Arbiritrary array x0 into x, with diffusion rate diff.
        float[,] diffuse(int N, boundary boundaryType, float[,] x, float[,] x0, float diff, float dt)
        {
            float diffusion_rate = dt * diff * N * N;   //Why is N^2 multiplied here?
            return lin_solve(N, boundaryType, x, x0, diffusion_rate, 1 + 4 * diffusion_rate); //Why are we dividing by 1 + 4*a?
        }

        //Advect step always had same params, so function has been simplified, and made less generic.
        void advect(int N, boundary boundaryType, ref float[,] d,ref float[,] d0,ref float[,] u,ref float[,] v, float dt)
        {

            float dt0 = dt * N;
            for (int i = 1; i <= N; i++)
            {
                for (int j = 1; j <= N; j++)
                {
                    float x = i - dt0 * u[i, j];
                    float y = j - dt0 * v[i, j];
                    if (x < 0.5f) x = 0.5f; if (x > N + 0.5f) x = N + 0.5f; int i0 = (int)x; int i1 = i0 + 1;
                    if (y < 0.5f) y = 0.5f; if (y > N + 0.5f) y = N + 0.5f; int j0 = (int)y; int j1 = j0 + 1;
                    float s1 = x - i0; float s0 = 1 - s1; float t1 = y - j0; float t0 = 1 - t1;
                    d[i, j] = s0 * (t0 * d0[i0, j0] + t1 * d0[i0, j1]) + s1 * (t0 * d0[i1, j0] + t1 * d0[i1, j1]);
                }
            }
            d = set_bnd(N, boundaryType, d);
        }

        //void project(int N, ref float[,] u, ref float[,] v, float[,] p , float[,] div)
        SolveState project(SolveState cur_state)
        {
            int N = cur_state.N;

            for (int i = 1; i <= N; i++)
            {
                for (int j = 1; j <= N; j++)
                {
                    cur_state.velocity_prevY[i, j] = -0.5f * (cur_state.velocityX[i + 1, j] - cur_state.velocityX[i - 1, j] + cur_state.velocityY[i, j + 1] - cur_state.velocityY[i, j - 1]) / N;
                    cur_state.density[i, j] = 0;
                }
            }
            cur_state.density = set_bnd(N, boundary.IGNORE, cur_state.density);
            cur_state.velocity_prevY = set_bnd(N, boundary.IGNORE, cur_state.velocity_prevY);
            cur_state.density = lin_solve(N, boundary.IGNORE, cur_state.density, cur_state.velocity_prevY, 1, 4);
            for (int i = 1; i <= N; i++)
            {
                for (int j = 1; j <= N; j++)
                {
                    cur_state.velocityX[i, j] -= 0.5f * N * (cur_state.density[i + 1, j] - cur_state.density[i - 1, j]);
                    cur_state.velocityY[i, j] -= 0.5f * N * (cur_state.density[i, j + 1] - cur_state.density[i, j - 1]);
                }
            }
            cur_state.velocityX = set_bnd(N, boundary.HORIZONTAL, cur_state.velocityX);
            cur_state.velocityY = set_bnd(N, boundary.VERTICAL, cur_state.velocityY);

            return cur_state;
        }

        //Density Step - Takes current state and returns next state
        public SolveState dens_step(SolveState cur_state)
        {
            int N = cur_state.N;
            float dt = cur_state.dt;
            float diff = cur_state.diff;

            cur_state.density = add_source(N, cur_state.density, cur_state.density_prev, dt);
            swap(ref cur_state.density_prev, ref cur_state.density);
            cur_state.density = diffuse(N, boundary.IGNORE, cur_state.density, cur_state.density_prev, diff, dt);
            swap(ref cur_state.density_prev, ref cur_state.density);
            advect(N, boundary.IGNORE, ref cur_state.density, ref cur_state.density_prev, ref cur_state.velocityX, ref cur_state.velocityY, dt);
            return cur_state;
        }

        //Velocity Step - Takes current state and returns next state
        public SolveState vel_step(SolveState cur_state)
        {
            int N = cur_state.N;
            float dt = cur_state.dt;
            float visc = cur_state.visc;

            cur_state.velocityX = add_source(N, cur_state.velocityX, cur_state.velocity_prevX, dt);
            cur_state.velocityY = add_source(N, cur_state.velocityY, cur_state.velocity_prevY, dt);

            swap(ref cur_state.velocity_prevX, ref cur_state.velocityX);
            cur_state.velocityX = diffuse(N, boundary.HORIZONTAL, cur_state.velocityX, cur_state.velocity_prevX, visc, dt);
            swap(ref cur_state.velocity_prevY, ref cur_state.velocityY);
            cur_state.velocityY = diffuse(N, boundary.VERTICAL, cur_state.velocityY, cur_state.velocity_prevY, visc, dt);

            cur_state = project(cur_state);

            swap(ref cur_state.velocity_prevX, ref cur_state.velocityX);
            swap(ref cur_state.velocity_prevY, ref cur_state.velocityY);

            advect(N, boundary.HORIZONTAL, ref cur_state.velocityX,ref cur_state.velocity_prevX,ref cur_state.velocity_prevX,ref cur_state.velocity_prevY, dt);
            advect(N, boundary.VERTICAL, ref cur_state.velocityY, ref cur_state.velocity_prevY, ref cur_state.velocity_prevX, ref cur_state.velocity_prevY, dt);

            cur_state = project(cur_state);
            return cur_state;
        }
    }
}