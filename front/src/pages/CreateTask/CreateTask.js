import React, {useState} from 'react';
import {Button, Card, Flex, Select, Textarea, TextInput, Title} from '@mantine/core';
import {isNotEmpty, useForm} from '@mantine/form';
import TaskList from '../../features/TaskList/TaskList';
import {useDispatch} from 'react-redux';
import {addTask} from "../../store/Slices/TaskSlice";
import {TASK_STATUSES} from "../../consts/tasks.consts";

const CreateTask = () => {
	const [robot, setRobot] = useState(null);
	const [priority, setPriority] = useState(null);
	const dispatch = useDispatch()

	const form = useForm({
		initialValues: {
			aisle: '',
			shelf: '',
			description: ''
		},
		validate: {
			aisle: isNotEmpty(),
			shelf: isNotEmpty(),
			description: isNotEmpty()
		},
	});

	const onSubmit = (values) => {
		if (!robot || !priority) {
			alert("make sure you select a robot and a priority")
			return;
		}
		console.log({...values, robot, priority});
		dispatch(addTask({...values, robot, priority, id: crypto.randomUUID(), status: TASK_STATUSES.DONE}));
	}

	return (
		<Flex justify="center">
			<Card miw="38rem" withBorder radius="1rem">
				<Title>Create new task</Title>
				<form onSubmit={form.onSubmit((values) => onSubmit(values))}>
					<Flex py="1rem" miw="35rem" justify="space-between">
						<Flex direction="column">
							<TextInput label="Aisle" withAsterisk
									   placeholder="Aisle Number" {...form.getInputProps('aisle')} />
							<TextInput my="xl" withAsterisk label="Shelf"
									   placeholder="Shelf Number" {...form.getInputProps('shelf')} />
							<Select
								label="Your favorite framework/library"
								placeholder="Pick one"
								value={robot}
								withAsterisk
								my="xl"
								onChange={setRobot}
								data={[
									{value: 'Robot A', label: 'Robot A'},
									{value: 'Robot B', label: 'Robot B'},
									{value: 'Robot C', label: 'Robot C'}
								]}
							/>
						</Flex>
						<Flex direction="column">
							<Select
								withAsterisk
								label="Your favorite framework/library"
								placeholder="Pick one"
								value={priority}
								onChange={setPriority}
								data={[
									{value: 'high', label: 'High'},
									{value: 'medium', label: 'Medium'},
									{value: 'low', label: 'Low'}
								]}
							/>
							<Textarea
								my="xl"
								placeholder="Your description"
								label="Description"
								withAsterisk
								minRows={5}
								{...form.getInputProps('description')}
							/>
						</Flex>
					</Flex>
					<Flex justify="center">
						<Button type="submit" w="9rem">Done</Button>
					</Flex>
				</form>
			</Card>
			<TaskList/>
		</Flex>
	);
};

export default CreateTask;
