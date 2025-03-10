#ifndef SMARACT_MCS_MOTOR_DRIVER_H
#define SMARACT_MCS_MOTOR_DRIVER_H

/* Motor driver support for smarAct MCS RS-232 Controller   */

/* Derived from ACRMotorDriver.cpp by Mark Rivers, 2011/3/28 */

/* Author: Till Straumann <strauman@slac.stanford.edu>, 9/11 */

#include <regex>
#ifdef __cplusplus

#include <asynMotorController.h>
#include <asynMotorAxis.h>
#include <stdarg.h>
#include <exception>

enum SmarActMCSExceptionType {
	MCSUnknownError,
	MCSConnectionError,
	MCSCommunicationError,
};

static constexpr char FREQUENCY_STRING[] = "FREQUENCY";
static constexpr char AMPLITUDE_STRING[] = "AMPLITUDE";
constexpr int MIN_AMPLITUDE = 0;
constexpr int MAX_AMPLITUDE = 4095;
constexpr int MIN_FREQUENCY = 1;
constexpr int MAX_FREQUENCY = 18500;
constexpr int DEFAULT_FREQUENCY = 100;
constexpr int DEFAULT_AMPLITUDE = MAX_AMPLITUDE;

class SmarActMCSException : public std::exception {
public:
	SmarActMCSException(SmarActMCSExceptionType t, const char *fmt, ...);
	SmarActMCSException(SmarActMCSExceptionType t)
		: t_(t)
		{ str_[0] = 0; }
	SmarActMCSException()
		: t_(MCSUnknownError)
		{ str_[0] = 0; }
	SmarActMCSException(SmarActMCSExceptionType t, const char *fmt, va_list ap);
	SmarActMCSExceptionType getType()
		const { return t_; }
	virtual const char *what()
		const throw() {return str_ ;}
protected:
	char str_[100];	
	SmarActMCSExceptionType t_;
};


class SmarActMCSAxis : public asynMotorAxis
{
public:
	SmarActMCSAxis(class SmarActMCSController *cnt_p, int axis, int channel);
	asynStatus  poll(bool *moving_p);
	asynStatus  move(double position, int relative, double min_vel, double max_vel, double accel);
	asynStatus  home(double min_vel, double max_vel, double accel, int forwards);
	asynStatus  stop(double acceleration);
	asynStatus  setPosition(double position);
	asynStatus  moveVelocity(double min_vel, double max_vel, double accel);

	virtual asynStatus getVal(const char *parm, int *val_p);
	virtual asynStatus getAngle(int *val_p, int *rev_p);
	virtual asynStatus moveCmd(const char *cmd, ...);
	virtual int        getClosedLoop();
	int        getEncoder();

	int         getVel() const { return vel_; }
	
protected:
	asynStatus  setSpeed(double velocity);
private:
	SmarActMCSController   *c_p_;  // pointer to asynMotorController for this axis
	asynStatus             comStatus_;
	int                    vel_;
	unsigned               holdTime_;
	int                    channel_;
	int                    sensorType_;
	int                    isRot_;
	int                    stepCount_; // open loop current step count
	int                    amplitude_ = DEFAULT_AMPLITUDE;
	int                    frequency_ = DEFAULT_FREQUENCY;

friend class SmarActMCSController;
};

class SmarActMCSController : public asynMotorController
{
public:
	SmarActMCSController(const char *portName, const char *IOPortName, int numAxes, double movingPollPeriod, double idlePollPeriod, int disableSpeed = 0);
	virtual asynStatus sendCmd(size_t *got_p, char *rep, int len, double timeout, const char *fmt, va_list ap);
	virtual asynStatus sendCmd(size_t *got_p, char *rep, int len, double timeout, const char *fmt, ...);
	virtual asynStatus sendCmd(size_t *got_p, char *rep, int len, const char *fmt, ...);
	virtual asynStatus sendCmd(char *rep, int len, const char *fmt, ...);

	virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
	virtual SmarActMCSAxis *getAxis(asynUser *pasynUser);
	virtual SmarActMCSAxis *getAxis(int axisNo);

	static int parseReply(const char *reply, int *ax_p, int *val_p);
	static int parseAngle(const char *reply, int *ax_p, int *val_p, int *rot_p);

protected:
	SmarActMCSAxis **pAxes_;

#define FIRST_MCS_PARAM driveFrequencyIndex_
	int driveFrequencyIndex_;
	int driveAmplitudeIndex_;
#define LAST_MCS_PARAM driveAmplitudeIndex_

private:
	asynUser *asynUserMot_p_;
	int disableSpeed_;
friend class SmarActMCSAxis;
};

#define NUM_MCS_PARAMS ((int)(&LAST_MCS_PARAM - &FIRST_MCS_PARAM + 1))

#endif // _cplusplus
#endif // SMARACT_MCS_MOTOR_DRIVER_H
