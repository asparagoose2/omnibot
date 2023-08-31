import React from 'react';
import {Badge, Flex, Text} from "@mantine/core";
import { STATUS_COLORS } from '../../consts/tasks.consts';

const TaskItem = ({status, aisle, shelf, robot}) => {
	return (
		<Flex miw="19rem" mb="1rem" justify="space-between" align="center">
			<Flex align="center">
				<Badge variant="filled" color={STATUS_COLORS[status] ?? 'gray'}></Badge>
				<Text ml="0.5rem">Aisle {aisle}</Text>
				<Text ml="0.5rem">Shelf {shelf}</Text>
			</Flex>
			<Text color={"dimmed"}>{robot}</Text>
		</Flex>
	);
};

export default TaskItem;
