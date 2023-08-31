export const TASK_STATUSES = {
	IDLE: 'IDLE',
	HANDLING: 'HANDLING',
	DONE: 'DONE',
	FAILED: 'FAILED',
}

export const STATUS_COLORS = {
	[TASK_STATUSES.IDLE]: 'gray',
	[TASK_STATUSES.HANDLING]: 'blue',
	[TASK_STATUSES.DONE]: 'green',
	[TASK_STATUSES.FAILED]: 'red',
}
