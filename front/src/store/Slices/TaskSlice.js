import {createSlice} from '@reduxjs/toolkit'

const initialState = []

// {
// 	id: 1,
// 	aisle: 'Aisle 6',
//	shelf: 'Shelf 1',
// 	description: 'Description 1',
// 	robot: 'Robot A',
// 	status: 'New',
// 	createdAt: '2021-01-01',
// 	updatedAt: '2021-01-01',
// }

export const taskSlice = createSlice({
	name: 'tasks',
	initialState,
	reducers: {
		addTask: (state, action) => {
			// Redux Toolkit allows us to write "mutating" logic in reducers. It
			// doesn't actually mutate the state because it uses the Immer library,
			// which detects changes to a "draft state" and produces a brand new
			// immutable state based off those changes
			return [...state, action.payload]
		},
		removeTask: (state, action) => {
			return state.filter(task => task.id !== action.payload)
		},
		updateTask: (state, action) => {
			const {id, ...rest} = action.payload
			const task = state.find(task => task.id === id)
			Object.assign(task, rest)
		},
	},
})

// Action creators are generated for each case reducer function
export const {addTask, removeTask, updateTask} = taskSlice.actions

export default taskSlice.reducer
