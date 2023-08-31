import { configureStore } from '@reduxjs/toolkit'
import taskReducer from './Slices/TaskSlice';

export const store = configureStore({
	reducer: {
		tasks: taskReducer,
	},
})
