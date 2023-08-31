import React from 'react';
import { Card, Flex } from '@mantine/core';
import { Controller, LiveCamera } from './components';

const Robot = () => {
	return (
		<Flex h="100%" direction="column">
				<iframe src="http://omnibot:8888" width="100%" height="100%" style={{border: "none"}}></iframe>
		</Flex>
	);
};

export default Robot;
