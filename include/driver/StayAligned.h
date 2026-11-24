// SPDX-License-Identifier: AGPL-3.0-only
// Added by Shinyflvres, 2026-09-29. Part of SpaceSync, a modified version of OpenVR-SpaceOverride by Nyabsi (AGPL-3.0). See NOTICE.md

#pragma once

#include "PoseMath.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <vector>

namespace stay
{
	struct Frame
	{
		double yaw = 0.0;
		vr::HmdVector3d_t t = { 0, 0, 0 };
	};

	inline vr::HmdVector3d_t RotY(double yaw, const vr::HmdVector3d_t& v)
	{
		double c = std::cos(yaw), s = std::sin(yaw);
		return { c * v.v[0] + s * v.v[2], v.v[1], -s * v.v[0] + c * v.v[2] };
	}

	inline Frame Mul(const Frame& a, const Frame& b) { return { wrapRad(a.yaw + b.yaw), vecAdd(a.t, RotY(a.yaw, b.t)) }; }
	inline Frame Inv(const Frame& a) { return { -a.yaw, RotY(-a.yaw, vecScale(a.t, -1.0)) }; }
	inline vr::HmdVector3d_t Apply(const Frame& f, const vr::HmdVector3d_t& p) { return vecAdd(f.t, RotY(f.yaw, p)); }
	inline double NormH(const vr::HmdVector3d_t& v) { return std::sqrt(v.v[0] * v.v[0] + v.v[2] * v.v[2]); }
	inline double Deg(double r) { return r * 180.0 / POSE_PI; }
	inline double Rad(double d) { return d * POSE_PI / 180.0; }

	struct Sample
	{
		double t = 0.0;
		double yaw = 0.0;
		vr::HmdVector3d_t p = { 0, 0, 0 };
	};

	struct StepFit
	{
		double T = 0.0;
		double yaw = 0.0;
		vr::HmdVector3d_t d = { 0, 0, 0 };
		double yawBefore = 0.0;
		vr::HmdVector3d_t before = { 0, 0, 0 };
		bool spikeBetter = false;
	};

	class StepDetector
	{
	public:
		void reset()
		{
			clearWindow();
			for (auto& h : noiseHist)
				h.clear();
		}

		void clearWindow()
		{
			buf.clear();
			fits.clear();
			cooldown = 0;
		}

		void shift(const Frame& J)
		{
			for (auto& s : buf)
			{
				s.yaw = wrapRad(s.yaw + J.yaw);
				s.p = Apply(J, s.p);
			}
			fits.clear();
		}

		bool push(const Sample& s, StepFit& out)
		{
			buf.push_back(s);
			if (buf.size() > 2 * Nside + 8)
				buf.pop_front();
			if (cooldown > 0)
			{
				cooldown--;
				return false;
			}
			if (buf.size() < 2 * Nside + 1)
				return false;

			size_t j = buf.size() - Nside;
			double x[4][5], rss[4], v;
			if (!fit(buf, j, false, x, rss, v))
				return false;

			double sig2[4];
			double T = 0.0;
			for (int ch = 0; ch < 4; ch++)
			{
				double local = rss[ch] / (2 * Nside - 5);
				double hist = 0.0;
				for (double h : noiseHist[ch])
					hist += h;
				hist = noiseHist[ch].empty() ? 0.0 : hist / noiseHist[ch].size();
				sig2[ch] = (std::max)((std::max)(sigmaNom[ch] * sigmaNom[ch], hist), local);
				T += x[ch][4] * x[ch][4] / (v * sig2[ch]);
				noiseHist[ch].push_back(local);
				if (noiseHist[ch].size() > 90)
					noiseHist[ch].pop_front();
			}

			StepFit f;
			f.T = T;
			f.yaw = x[0][4];

			double xs[4][5], rssS[4], vs;
			double rssStep = 0.0, rssSpike = 0.0;
			if (fit(buf, j, true, xs, rssS, vs))
			{
				for (int ch = 0; ch < 4; ch++)
				{
					rssStep += rss[ch] / sig2[ch];
					rssSpike += rssS[ch] / sig2[ch];
				}
				f.spikeBetter = rssSpike < rssStep;
			}

			f.yawBefore = wrapRad(buf[j - 1].yaw + x[0][0]);
			f.before = { x[1][0], x[2][0], x[3][0] };

			std::deque<Sample> rot = buf;
			for (size_t k = j; k < rot.size(); k++)
				rot[k].p = RotY(-f.yaw, rot[k].p);
			double xr[4][5], rssr[4], vr_;
			vr::HmdVector3d_t stepQ = { x[1][4], x[2][4], x[3][4] };
			if (fit(rot, j, false, xr, rssr, vr_))
				stepQ = { xr[1][4], xr[2][4], xr[3][4] };
			f.d = RotY(f.yaw, stepQ);

			fits.push_back(f);
			if (fits.size() > 7)
				fits.erase(fits.begin());
			if (fits.size() < 7)
				return false;

			const StepFit& c = fits[3];
			bool localMax = true;
			for (int k = 0; k < 7; k++)
				if (k != 3 && fits[k].T > c.T)
					localMax = false;
			bool size = std::fabs(c.yaw) > Rad(0.3) || NormH(c.d) > 0.005;
			if (c.T > 40.0 && localMax && size && !c.spikeBetter)
			{
				out = c;
				cooldown = 2 * Nside;
				Frame J = { c.yaw, c.d };
				size_t split = buf.size() - Nside - 3;
				for (size_t k = 0; k < split; k++)
				{
					buf[k].yaw = wrapRad(buf[k].yaw + J.yaw);
					buf[k].p = Apply(J, buf[k].p);
				}
				fits.clear();
				return true;
			}
			return false;
		}

	private:
		static const int Nside = 6;
		std::deque<Sample> buf;
		std::deque<double> noiseHist[4];
		std::vector<StepFit> fits;
		int cooldown = 0;
		const double sigmaNom[4] = { 0.05 * POSE_PI / 180.0, 0.0005, 0.0005, 0.0005 };

		static bool invert5(double A[5][5], double inv[5][5])
		{
			double M[5][10];
			for (int r = 0; r < 5; r++)
				for (int c = 0; c < 10; c++)
					M[r][c] = c < 5 ? A[r][c] : (c - 5 == r ? 1.0 : 0.0);
			for (int c = 0; c < 5; c++)
			{
				int piv = c;
				for (int r = c + 1; r < 5; r++)
					if (std::fabs(M[r][c]) > std::fabs(M[piv][c]))
						piv = r;
				if (std::fabs(M[piv][c]) < 1e-12)
					return false;
				for (int k = 0; k < 10; k++)
					std::swap(M[c][k], M[piv][k]);
				double d = M[c][c];
				for (int k = 0; k < 10; k++)
					M[c][k] /= d;
				for (int r = 0; r < 5; r++)
				{
					if (r == c)
						continue;
					double f = M[r][c];
					for (int k = 0; k < 10; k++)
						M[r][k] -= f * M[c][k];
				}
			}
			for (int r = 0; r < 5; r++)
				for (int c = 0; c < 5; c++)
					inv[r][c] = M[r][c + 5];
			return true;
		}

		static bool fit(const std::deque<Sample>& b, size_t j, bool spikeModel, double xOut[4][5], double rss[4], double& v)
		{
			size_t lo = j - Nside, hi = j + Nside;
			double tm = 0.5 * (b[j - 1].t + b[j].t);
			double span = 0.5 * (b[hi - 1].t - b[lo].t);
			if (!(span > 1e-6))
				return false;
			double A[5][5] = {};
			double X[2 * Nside][5];
			for (size_t k = lo; k < hi; k++)
			{
				double tau = (b[k].t - tm) / span;
				double row[5] = { 1.0, tau, tau * tau, tau * tau * tau, spikeModel ? (k == j ? 1.0 : 0.0) : (k >= j ? 1.0 : 0.0) };
				for (int a = 0; a < 5; a++)
				{
					X[k - lo][a] = row[a];
					for (int c = 0; c < 5; c++)
						A[a][c] += row[a] * row[c];
				}
			}
			double inv[5][5];
			if (!invert5(A, inv))
				return false;
			v = inv[4][4];
			double base = b[j - 1].yaw;
			for (int ch = 0; ch < 4; ch++)
			{
				double rhs[5] = {};
				double ys[2 * Nside];
				for (size_t k = lo; k < hi; k++)
				{
					double y = ch == 0 ? wrapRad(b[k].yaw - base) : b[k].p.v[ch - 1];
					ys[k - lo] = y;
					for (int a = 0; a < 5; a++)
						rhs[a] += X[k - lo][a] * y;
				}
				for (int a = 0; a < 5; a++)
				{
					xOut[ch][a] = 0.0;
					for (int c = 0; c < 5; c++)
						xOut[ch][a] += inv[a][c] * rhs[c];
				}
				double s = 0.0;
				for (int r = 0; r < 2 * Nside; r++)
				{
					double f = 0.0;
					for (int a = 0; a < 5; a++)
						f += X[r][a] * xOut[ch][a];
					s += (ys[r] - f) * (ys[r] - f);
				}
				rss[ch] = s;
			}
			return true;
		}
	};

	struct Ekf3
	{
		Frame C;
		double P[3][3] = {};
		double G[3][3] = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };
		int accepted = 0, rejected = 0;
		double lastInnovation = 0.0;

		static void Sandwich(const double A[3][3], double X[3][3])
		{
			double T[3][3], O[3][3];
			for (int r = 0; r < 3; r++)
				for (int c = 0; c < 3; c++)
				{
					T[r][c] = 0.0;
					for (int k = 0; k < 3; k++)
						T[r][c] += A[r][k] * X[k][c];
				}
			for (int r = 0; r < 3; r++)
				for (int c = 0; c < 3; c++)
				{
					O[r][c] = 0.0;
					for (int k = 0; k < 3; k++)
						O[r][c] += T[r][k] * A[c][k];
				}
			for (int r = 0; r < 3; r++)
				for (int c = 0; c < 3; c++)
					X[r][c] = 0.5 * (O[r][c] + O[c][r]);
		}

		static void Adjoint(const Frame& J, const Frame& Cc, double A[3][3])
		{
			vr::HmdVector3d_t v = RotY(Cc.yaw, J.t);
			double c = std::cos(J.yaw), s = std::sin(J.yaw);
			double M[3][3] = { { c, s, -v.v[2] }, { -s, c, v.v[0] }, { 0, 0, 1 } };
			for (int r = 0; r < 3; r++)
				for (int k = 0; k < 3; k++)
					A[r][k] = M[r][k];
		}

		void reset(double sp, double sy)
		{
			C = Frame{};
			for (int r = 0; r < 3; r++)
				for (int c = 0; c < 3; c++)
				{
					P[r][c] = 0.0;
					G[r][c] = r == c ? 1.0 : 0.0;
				}
			P[0][0] = P[1][1] = sp * sp;
			P[2][2] = sy * sy;
			accepted = rejected = 0;
		}

		void resetG()
		{
			for (int r = 0; r < 3; r++)
				for (int c = 0; c < 3; c++)
					G[r][c] = r == c ? 1.0 : 0.0;
		}

		void follow(const Frame& J)
		{
			double A[3][3];
			Adjoint(J, C, A);
			Sandwich(A, P);
			double A0[3][3];
			Adjoint(J, Frame{}, A0);
			double Gn[3][3];
			for (int r = 0; r < 3; r++)
				for (int c = 0; c < 3; c++)
				{
					Gn[r][c] = 0.0;
					for (int k = 0; k < 3; k++)
						Gn[r][c] += A0[r][k] * G[k][c];
				}
			for (int r = 0; r < 3; r++)
				for (int c = 0; c < 3; c++)
					G[r][c] = Gn[r][c];
			C = Mul(J, Mul(C, Inv(J)));
			C.t.v[1] = 0.0;
		}

		void decay(double a, double sp2, double sy2)
		{
			C.yaw *= a;
			C.t = vecScale(C.t, a);
			double Q[3][3] = { { sp2, 0, 0 }, { 0, sp2, 0 }, { 0, 0, sy2 } };
			Sandwich(G, Q);
			for (int r = 0; r < 3; r++)
				for (int c = 0; c < 3; c++)
					P[r][c] = a * a * P[r][c] + (1.0 - a * a) * Q[r][c];
		}

		bool update(const double y[2], const double H[2][3], double Rm, double stepMaxPos, double stepMaxYaw)
		{
			double PHt[3][2];
			for (int r = 0; r < 3; r++)
				for (int c = 0; c < 2; c++)
				{
					PHt[r][c] = 0.0;
					for (int k = 0; k < 3; k++)
						PHt[r][c] += P[r][k] * H[c][k];
				}
			double S[2][2];
			for (int r = 0; r < 2; r++)
				for (int c = 0; c < 2; c++)
				{
					S[r][c] = r == c ? Rm : 0.0;
					for (int k = 0; k < 3; k++)
						S[r][c] += H[r][k] * PHt[k][c];
				}
			double det = S[0][0] * S[1][1] - S[0][1] * S[1][0];
			if (!(det > 0.0))
				return false;
			double Si[2][2] = { { S[1][1] / det, -S[0][1] / det }, { -S[1][0] / det, S[0][0] / det } };
			double m = y[0] * (Si[0][0] * y[0] + Si[0][1] * y[1]) + y[1] * (Si[1][0] * y[0] + Si[1][1] * y[1]);
			lastInnovation = std::sqrt(y[0] * y[0] + y[1] * y[1]);
			if (m > 13.8)
			{
				rejected++;
				return false;
			}
			accepted++;
			double K[3][2];
			for (int r = 0; r < 3; r++)
				for (int c = 0; c < 2; c++)
					K[r][c] = PHt[r][0] * Si[0][c] + PHt[r][1] * Si[1][c];
			double dx[3];
			for (int r = 0; r < 3; r++)
				dx[r] = K[r][0] * y[0] + K[r][1] * y[1];
			double moveX = dx[0] + H[0][2] * dx[2], moveZ = dx[1] + H[1][2] * dx[2];
			double move = std::sqrt(moveX * moveX + moveZ * moveZ);
			double scale = 1.0;
			if (move > stepMaxPos)
				scale = stepMaxPos / move;
			if (std::fabs(dx[2]) * scale > stepMaxYaw)
				scale = stepMaxYaw / std::fabs(dx[2]);
			C.t.v[0] += dx[0] * scale;
			C.t.v[2] += dx[1] * scale;
			C.yaw = wrapRad(C.yaw + dx[2] * scale);
			for (int r = 0; r < 3; r++)
				for (int c = 0; c < 2; c++)
					K[r][c] *= scale;
			double A[3][3];
			for (int r = 0; r < 3; r++)
				for (int c = 0; c < 3; c++)
					A[r][c] = (r == c ? 1.0 : 0.0) - (K[r][0] * H[0][c] + K[r][1] * H[1][c]);
			double AP[3][3];
			for (int r = 0; r < 3; r++)
				for (int c = 0; c < 3; c++)
				{
					AP[r][c] = 0.0;
					for (int k = 0; k < 3; k++)
						AP[r][c] += A[r][k] * P[k][c];
				}
			double Pn[3][3];
			for (int r = 0; r < 3; r++)
				for (int c = 0; c < 3; c++)
				{
					Pn[r][c] = Rm * (K[r][0] * K[c][0] + K[r][1] * K[c][1]);
					for (int k = 0; k < 3; k++)
						Pn[r][c] += AP[r][k] * A[c][k];
				}
			for (int r = 0; r < 3; r++)
				for (int c = 0; c < 3; c++)
					P[r][c] = 0.5 * (Pn[r][c] + Pn[c][r]);
			return true;
		}
	};

	struct Hip
	{
		bool valid = false;
		vr::HmdVector3d_t position = { 0, 0, 0 };
		vr::HmdQuaternion_t rotation = { 1, 0, 0, 0 };
	};

	struct Event
	{
		bool jump = false;
		bool follow = false;
		bool reverted = false;
		// The classifier wanted to follow, but the caller flagged the jump as a
		// likely SLAM relocalization, so it took the held path instead.
		bool postReloc = false;
		Frame J;
		double T = 0.0;
		double lambda = 0.0;
		double yawAfterDeg = 0.0;
		bool rebase = false;
		int rebaseKind = 0;
		Frame K;
		double rescueYawMisDeg = 0.0;
		double rescueTransMisM = 0.0;
		bool rescueCanonical = false;
		bool rescueChecked = false;
	};

	class Aligner
	{
	public:
		uint32_t recentersFollowed = 0;
		uint32_t jumpsHeld = 0;
		uint32_t rescues = 0;

		void reset()
		{
			det.reset();
			ekf.reset(0.005, Rad(0.3));
			primedHead = false;
			lastSampleT = -1.0;
			lastGoodT = -1.0;
			lastBodyT = -1.0;
			standY = 0.0;
			standValid = false;
			emaY = 0.0;
			speed = 0.0;
			sitSince = 0.0;
			baseOff = { 0, 0, 0 };
			baseW = 0.0;
			baseT = 0.0;
			ready = false;
			bodyStart = -1.0;
			baseOffSit = { 0, 0, 0 };
			baseWSit = 0.0;
			baseTSit = 0.0;
			hhSin = hhCos = 0.0;
			hhW = 0.0;
			hipAxis = -1;
			accumReset(0.0);
			heldT = -1e9;
			rescueMode = false;
			postGap = false;
			pgClear();
			rejectRun = 0;
			recentersFollowed = jumpsHeld = rescues = 0;
		}

		const Frame& correction() const { return ekf.C; }
		bool bodyReady() const { return ready; }
		double yawSigmaDeg() const { return Deg(std::sqrt((std::max)(0.0, ekf.P[2][2]))); }
		int accepted() const { return ekf.accepted; }
		int rejected() const { return ekf.rejected; }
		bool inRescue() const { return rescueMode; }
		bool waitingAfterGap() const { return postGap; }

		void externalShift(double t, const Frame& J)
		{
			det.shift(J);
			ekf.follow(J);
			standY += J.t.v[1];
			emaY += J.t.v[1];
			heldT = -1e9;
			pauseBody(t);
		}

		void resetBody()
		{
			baseOff = { 0, 0, 0 };
			baseW = baseT = 0.0;
			ready = false;
			bodyStart = -1.0;
			baseOffSit = { 0, 0, 0 };
			baseWSit = baseTSit = 0.0;
			hhSin = hhCos = hhW = 0.0;
			hipAxis = -1;
			standValid = false;
			postGap = false;
			pgClear();
			rejectRun = 0;
			accumReset(0.0);
		}

		Event step(double t, const vr::HmdQuaternion_t& headRot, const vr::HmdVector3d_t& headPos, const Hip& hip, bool suspectReloc = false)
		{
			Event ev;
			double headYaw = quaternionYawRad(headRot);
			vr::HmdVector3d_t fwd = quaternionRotateVector(headRot, vr::HmdVector3d_t{ 0.0, 0.0, -1.0 });
			double horizontal = std::sqrt(fwd.v[0] * fwd.v[0] + fwd.v[2] * fwd.v[2]);
			bool pitchOk = horizontal > 0.5;

			if (primedHead && headYaw == lastYaw && headPos.v[0] == lastPos.v[0] && headPos.v[1] == lastPos.v[1] && headPos.v[2] == lastPos.v[2])
				return ev;

			double gap = lastGoodT < 0.0 ? 0.0 : t - lastGoodT;
			if (gap > 2.0 && bodyReady())
				startRescueCheck(t, headYaw, headPos, false);
			lastGoodT = t;

			if (!pitchOk)
			{
				det.clearWindow();
				lastSampleT = -1.0;
			}
			else
			{
				if (lastSampleT >= 0.0 && t - lastSampleT > 0.25)
					det.clearWindow();
				lastSampleT = t;

				StepFit f;
				if (det.push({ t, headYaw, headPos }, f))
				{
					ev.jump = true;
					ev.J = { f.yaw, f.d };
					ev.T = f.T;
					handleJump(t, f, ev, suspectReloc);
				}
			}

			if (primedHead)
			{
				double dt = t - lastHeadT;
				if (dt > 0.0 && dt < 0.5)
				{
					double inst = NormH(vecSub(headPos, lastPos)) / dt;
					double k = 1.0 - std::exp(-dt / 0.1);
					speed += ((std::min)(inst, 5.0) - speed) * k;
					double ky = 1.0 - std::exp(-dt / 2.0);
					emaY += (headPos.v[1] - emaY) * ky;
				}
			}
			else
				emaY = headPos.v[1];
			primedHead = true;
			lastHeadT = t;
			lastYaw = headYaw;
			lastPos = headPos;

			if (ev.follow)
				pauseBody(t);
			body(t, headYaw, headPos, horizontal > 0.85, hip, ev);
			if (ev.rebase)
				pauseBody(t);
			return ev;
		}

	private:
		StepDetector det;
		Ekf3 ekf;

		const double baseNeeded = 60.0;
		const double driftTau = 300.0;
		const double driftPos = 0.04;
		const double driftYaw = 1.0 * POSE_PI / 180.0;

		bool primedHead = false;
		double lastHeadT = 0.0;
		double lastYaw = 0.0;
		vr::HmdVector3d_t lastPos = { 0, 0, 0 };
		double lastSampleT = -1.0;
		double lastGoodT = -1.0;
		double lastBodyT = -1.0;

		double standY = 0.0;
		bool standValid = false;
		double emaY = 0.0;
		double speed = 0.0;
		double sitSince = 0.0;

		vr::HmdVector3d_t baseOff = { 0, 0, 0 };
		double baseW = 0.0, baseT = 0.0;
		bool ready = false;
		double bodyStart = -1.0;
		vr::HmdVector3d_t baseOffSit = { 0, 0, 0 };
		double baseWSit = 0.0, baseTSit = 0.0;

		double hhSin = 0.0, hhCos = 0.0, hhW = 0.0;
		int hipAxis = -1;

		double accY[2] = { 0, 0 };
		double accH[2][3] = {};
		int accN = 0;
		double accT = 0.0;
		double accDur = 0.0;

		double heldT = -1e9;
		Frame heldJ, heldC;

		bool rescueMode = false;
		bool postGap = false;
		bool pgCanonical = false;
		double pgStart = 0.0;
		double pgSin = 0.0, pgCos = 0.0;
		double pgDur = 0.0;
		vr::HmdVector3d_t pgY = { 0, 0, 0 };
		std::vector<vr::HmdVector3d_t> pgExp, pgHip;
		int rejectRun = 0;
		double bodyPausedUntil = -1.0;

		void pauseBody(double t)
		{
			bodyPausedUntil = t + 0.15;
			accumReset(t);
		}

		void accumReset(double t)
		{
			accY[0] = accY[1] = 0.0;
			for (int a = 0; a < 2; a++)
				for (int c = 0; c < 3; c++)
					accH[a][c] = 0.0;
			accN = 0;
			accT = t;
			accDur = 0.0;
		}

		void pgClear()
		{
			pgSin = pgCos = 0.0;
			pgDur = 0.0;
			pgY = { 0, 0, 0 };
			pgExp.clear();
			pgHip.clear();
		}

		void startRescueCheck(double t, double headYaw, const vr::HmdVector3d_t& headPos, bool watchdog)
		{
			postGap = true;
			pgStart = t;
			pgClear();
			pgCanonical = !watchdog && std::fabs(headYaw) < Rad(10.0) && NormH(headPos) < 0.25;
			accumReset(t);
			rejectRun = 0;
		}

		void handleJump(double t, const StepFit& f, Event& ev, bool suspectReloc)
		{
			const Frame& J = ev.J;
			vr::HmdVector3d_t headPred = f.before;
			vr::HmdVector3d_t dHead = vecSub(Apply(J, headPred), headPred);
			ev.yawAfterDeg = Deg(wrapRad(f.yawBefore + J.yaw));

			const double sRlY = Rad(2.0), sRlP = 0.08, sRcY = Rad(3.0), sRcP = 0.05;
			double psiRc = wrapRad(-f.yawBefore);
			vr::HmdVector3d_t hxz = { headPred.v[0], 0.0, headPred.v[2] };
			auto mahaRc = [&](bool canonical)
			{
				vr::HmdVector3d_t dRc = vecSub(canonical ? vr::HmdVector3d_t{ 0, 0, 0 } : hxz, RotY(psiRc, hxz));
				vr::HmdVector3d_t dRcHead = vecSub(Apply(Frame{ psiRc, dRc }, headPred), headPred);
				double ey = wrapRad(J.yaw - psiRc);
				vr::HmdVector3d_t ep = vecSub(dHead, dRcHead);
				return (ey / sRcY) * (ey / sRcY) + (ep.v[0] * ep.v[0] + ep.v[2] * ep.v[2]) / (sRcP * sRcP);
			};
			double mRl = (J.yaw / sRlY) * (J.yaw / sRlY) + (dHead.v[0] * dHead.v[0] + dHead.v[2] * dHead.v[2]) / (sRlP * sRlP);
			double a = -0.5 * mahaRc(true), c = -0.5 * mahaRc(false);
			double mx = (std::max)(a, c);
			double llRc = mx + std::log(0.5 * std::exp(a - mx) + 0.5 * std::exp(c - mx));
			double logDet = std::log((sRcY * sRcP * sRcP) / (sRlY * sRlP * sRlP));
			ev.lambda = 0.5 * mRl + llRc - logDet + std::log(1.5 / 6.7);
			bool wantFollow = ev.lambda > 0.0 || std::fabs(J.yaw) > Rad(20.0);
			// Decided here, not by the caller: once step() returns, the follow
			// branch below has already moved the EKF and the stand-height refs.
			ev.postReloc = wantFollow && suspectReloc;
			ev.follow = wantFollow && !ev.postReloc;

			if (ev.follow)
			{
				recentersFollowed++;
				ekf.follow(J);
				standY += J.t.v[1];
				emaY += J.t.v[1];
				heldT = -1e9;
				return;
			}

			jumpsHeld++;
			Frame both = Mul(J, heldJ);
			bool revert = t - heldT < 90.0 && std::fabs(Deg(both.yaw)) < 1.0 && NormH(vecSub(Apply(both, headPred), headPred)) < 0.03;
			if (revert)
			{
				ekf.C = heldC;
				heldT = -1e9;
				ev.reverted = true;
				return;
			}
			heldT = t;
			heldJ = J;
			heldC = ekf.C;
			vr::HmdVector3d_t cHead = vecSub(Apply(ekf.C, headPred), headPred);
			double dd = dHead.v[0] * dHead.v[0] + dHead.v[2] * dHead.v[2];
			if (dd > 1e-8)
			{
				double alpha = -(cHead.v[0] * dHead.v[0] + cHead.v[2] * dHead.v[2]) / dd;
				alpha = (std::max)(0.0, (std::min)(1.0, alpha));
				ekf.C = Mul(Frame{ J.yaw * alpha, vr::HmdVector3d_t{ J.t.v[0] * alpha, 0.0, J.t.v[2] * alpha } }, ekf.C);
				ekf.C.t.v[1] = 0.0;
			}
		}

		double hipHeading(const vr::HmdQuaternion_t& q)
		{
			const vr::HmdVector3d_t axes[3] = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, -1 } };
			if (hipAxis < 0)
			{
				double best = 2.0;
				for (int i = 0; i < 3; i++)
				{
					double vy = std::fabs(quaternionRotateVector(q, axes[i]).v[1]);
					if (vy < best)
					{
						best = vy;
						hipAxis = i;
					}
				}
			}
			vr::HmdVector3d_t a = quaternionRotateVector(q, axes[hipAxis]);
			return std::atan2(-a.v[0], -a.v[2]);
		}

		void body(double t, double headYaw, const vr::HmdVector3d_t& headPos, bool level, const Hip& hip, Event& ev)
		{
			double dt = lastBodyT < 0.0 ? 0.0 : (std::min)(0.5, (std::max)(0.0, t - lastBodyT));
			lastBodyT = t;

			if (bodyReady())
			{
				if (rescueMode)
				{
					double q = 2.0 * dt / driftTau;
					ekf.P[0][0] += q * driftPos * driftPos;
					ekf.P[1][1] += q * driftPos * driftPos;
					ekf.P[2][2] += q * driftYaw * driftYaw;
					if (ekf.P[2][2] < driftYaw * driftYaw)
					{
						ev.rebase = true;
						ev.rebaseKind = 2;
						ev.K = ekf.C;
						ekf.C = Frame{};
						ekf.resetG();
						rescueMode = false;
					}
				}
				else if (dt > 0.0)
					ekf.decay(std::exp(-dt / driftTau), driftPos * driftPos, driftYaw * driftYaw);
			}

			if (!hip.valid || t < bodyPausedUntil)
				return;

			if (!standValid || emaY > standY + 0.2)
			{
				bool relearn = standValid;
				standY = emaY;
				standValid = true;
				if (relearn)
				{
					baseW = baseT = 0.0;
					baseOff = { 0, 0, 0 };
					ready = false;
					bodyStart = t;
				}
			}
			else if (emaY > standY)
				standY = emaY;
			else
				standY -= dt * 0.01 / 60.0;

			double dh = headPos.v[1] - standY;
			bool calm = speed < 0.3 && level;
			bool standing = std::fabs(dh) < 0.08 && calm;
			bool sitHeight = dh < -0.2 && dh > -0.75;
			if (!sitHeight)
				sitSince = t;
			bool sitting = sitHeight && t - sitSince >= 5.0 && calm && bodyReady();
			int cls = standing ? 0 : (sitting ? 1 : -1);

			vr::HmdVector3d_t hipW = hip.position;
			vr::HmdVector3d_t hipS = Apply(ekf.C, hipW);
			vr::HmdVector3d_t off = RotY(-headYaw, vecSub(hipS, headPos));
			off.v[1] = 0.0;

			if (!ready)
			{
				if (bodyStart < 0.0)
					bodyStart = t;
				if (cls == 0)
				{
					baseW += 1.0;
					baseT += dt;
					baseOff = vecAdd(baseOff, vecScale(vecSub(off, baseOff), 1.0 / baseW));
				}
				ready = baseT >= baseNeeded || (t - bodyStart >= 90.0 && baseT >= 15.0);
				return;
			}

			if (cls == 1 && !postGap && !rescueMode && ekf.P[2][2] < driftYaw * driftYaw && baseTSit < baseNeeded)
			{
				baseWSit += 1.0;
				baseTSit += dt;
				baseOffSit = vecAdd(baseOffSit, vecScale(vecSub(off, baseOffSit), 1.0 / baseWSit));
			}

			bool gate = cls == 0 || (cls == 1 && baseTSit >= baseNeeded);
			vr::HmdVector3d_t bo = (cls == 1 && baseTSit >= baseNeeded) ? baseOffSit : baseOff;
			vr::HmdVector3d_t expected = vecAdd(headPos, RotY(headYaw, bo));
			double hipYawS0 = hipHeading(hip.rotation);

			if (postGap)
			{
				if (gate || cls == 1)
				{
					double hhOff = std::atan2(hhSin, hhCos);
					double cand = wrapRad(headYaw - hipYawS0 - hhOff);
					pgSin += std::sin(cand);
					pgCos += std::cos(cand);
					pgExp.push_back(expected);
					pgHip.push_back(hipW);
					pgY = vecAdd(pgY, vecSub(expected, hipS));
					pgDur += dt;
					if (pgDur >= 3.0 && pgExp.size() >= 30)
						decideRescue(t, ev);
				}
				return;
			}

			if (cls == 0 && !rescueMode && ekf.P[2][2] < driftYaw * driftYaw)
			{
				double f = std::exp(-dt / 900.0);
				double o = wrapRad(headYaw - ekf.C.yaw - hipYawS0);
				hhSin = hhSin * f + std::sin(o);
				hhCos = hhCos * f + std::cos(o);
				hhW = hhW * f + 1.0;
			}

			if (!gate)
				return;

			vr::HmdVector3d_t rr = RotY(ekf.C.yaw, hipW);
			accY[0] += expected.v[0] - hipS.v[0];
			accY[1] += expected.v[2] - hipS.v[2];
			accH[0][0] += 1.0;
			accH[1][1] += 1.0;
			accH[0][2] += rr.v[2];
			accH[1][2] += -rr.v[0];
			accN++;
			accDur += dt;
			if (t - accT > 30.0 && accDur < 3.0)
			{
				accumReset(t);
				return;
			}
			if (accDur >= 3.0 && accN > 0)
			{
				double y[2] = { accY[0] / accN, accY[1] / accN };
				double H[2][3];
				for (int a = 0; a < 2; a++)
					for (int c = 0; c < 3; c++)
						H[a][c] = accH[a][c] / accN;
				double boost = ekf.P[2][2] > Rad(2.0) * Rad(2.0) ? 20.0 : 1.0;
				double span = (std::min)(t - accT, accDur + 3.0);
				bool ok = ekf.update(y, H, 0.03 * 0.03, boost * 0.001 * span, boost * Rad(0.03) * span);
				if (!ok && ekf.lastInnovation > 0.3)
					rejectRun++;
				else
					rejectRun = 0;
				accumReset(t);
				if (rejectRun >= 3)
					startRescueCheck(t, headYaw, headPos, true);
			}
		}

		void decideRescue(double t, Event& ev)
		{
			double yawNew = std::atan2(pgSin, pgCos);
			double yawMis = wrapRad(yawNew - ekf.C.yaw);
			vr::HmdVector3d_t meanY = vecScale(pgY, 1.0 / pgExp.size());
			double transMis = NormH(meanY);
			ev.rescueChecked = true;
			ev.rescueYawMisDeg = Deg(yawMis);
			ev.rescueTransMisM = transMis;
			ev.rescueCanonical = pgCanonical;
			postGap = false;
			accumReset(t);

			if (!(transMis > 0.30 || (pgCanonical && std::fabs(yawMis) > Rad(20.0)) || std::fabs(yawMis) > Rad(60.0)))
				return;

			vr::HmdVector3d_t tNew = { 0, 0, 0 }, x0 = { 0, 0, 0 };
			for (size_t k = 0; k < pgExp.size(); k++)
			{
				tNew = vecAdd(tNew, vecSub(pgExp[k], RotY(yawNew, pgHip[k])));
				x0 = vecAdd(x0, pgExp[k]);
			}
			tNew = vecScale(tNew, 1.0 / pgExp.size());
			x0 = vecScale(x0, 1.0 / pgExp.size());
			tNew.v[1] = 0.0;

			ev.rebase = true;
			ev.rebaseKind = 1;
			ev.K = Frame{ yawNew, tNew };
			ekf.C = Frame{};
			ekf.resetG();
			double g[3] = { -x0.v[2], x0.v[0], 1.0 };
			double sd2 = Rad(30.0) * Rad(30.0);
			for (int a = 0; a < 3; a++)
				for (int c = 0; c < 3; c++)
					ekf.P[a][c] = sd2 * g[a] * g[c];
			ekf.P[0][0] += 0.05 * 0.05;
			ekf.P[1][1] += 0.05 * 0.05;
			rescueMode = true;
			rescues++;
			heldT = -1e9;
		}
	};
}
