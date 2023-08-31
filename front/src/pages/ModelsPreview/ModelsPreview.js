import React from 'react';
import useSelectTasks from "../../hooks/useSelectTasks";
import {Card, Flex, Text, Title} from "@mantine/core";
import TaskList from "../../features/TaskList/TaskList";

const ModelsPreview = () => {
	const tasks = useSelectTasks();
	console.log(tasks)
	return (
		<Flex justify="center">
			<Card w="30rem" withBorder radius="md" miw="29rem">
				<Flex justify="space-between" align="center">
					<div>
						<Title>Robot Task</Title>
						<Text c="dimmed">Aisle B Shelf 6</Text>
					</div>
					<Text>Battery</Text>
				</Flex>
			</Card>

			<TaskList/>
		</Flex>
	);
};

export default ModelsPreview;
