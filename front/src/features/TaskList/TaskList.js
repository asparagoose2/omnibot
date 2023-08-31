import React from 'react';
import {Card, Text, Title} from "@mantine/core";
import TaskItem from "../TaskItem/TaskItem";
import useSelectTasks from "../../hooks/useSelectTasks";

const TaskList = () => {
	const tasks = useSelectTasks();
	console.log(tasks)
	return (
		<Card  withBorder radius="md" h="min-content" p="1rem" ml="12rem">
			<Title>Task List</Title>
			<Text c="dimmed" mb="1rem">Tasks in queue</Text>
			{tasks.length > 0 ? tasks.map(({robot, aisle, shelf, status, id}) => (
				<TaskItem key={id} robot={robot} aisle={aisle} shelf={shelf} status={status}/>
			)) : <Text w="19rem">No tasks!</Text>}
		</Card>
	);
};

export default TaskList;
