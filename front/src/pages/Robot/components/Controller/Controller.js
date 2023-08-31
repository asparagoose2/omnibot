import { useState, useCallback } from 'react';
import { ThemeIcon, SimpleGrid, Flex, Button, clsx, ActionIcon } from '@mantine/core';
import { ReactComponent as PowerButton } from '../../../../images/power-button.svg';
import { ReactComponent as ArrowUp } from '../../../../images/arrow-up.svg';
import { ReactComponent as ArrowDown } from '../../../../images/arrow-down.svg';
import { ReactComponent as ArrowRight } from '../../../../images/arrow-right.svg';
import { ReactComponent as ArrowLeft } from '../../../../images/arrow-left.svg';
import './controller.css';

const DIRECTIONS = {
    UP: 'UP',
    LEFT: 'LEFT',
    RIGHT: 'RIGHT',
    DOWN: 'DOWN'
};

const Controller = () => {
    const [isPowered, setIsPowered] = useState(false);

    const onPower = useCallback(() => {
        const data = new FormData();
        data.append("shouldStop",isPowered)
        fetch('http://omnibot:5556/',{
            method:'POST',
            body:data
        }).catch(err => {
            console.log(err)
        });
        setIsPowered(isCurrentlyPowered => {
            console.log(`powering ${isCurrentlyPowered ? 'off' : 'on'}`);
            return !isCurrentlyPowered;
        });
        
    }, [setIsPowered,isPowered]);

    const createOnKeyClicked = useCallback((direction) => () => {
        console.log(`move ${direction}`);
    }, []);

    const containerClasses = clsx('controller-container', {poweredOn: isPowered});

    return <Button onClick={onPower} variant='none' className='controller-button'>
    <ActionIcon color={isPowered ? 'green' : 'dark'}  size="lg" className="controller-icon">
        <PowerButton />
    </ActionIcon>
</Button>


    return (
        <Flex className={containerClasses}>
            <SimpleGrid cols={3}>
                <div />
                
                <Flex justify="center">
                    <Button onClick={createOnKeyClicked(DIRECTIONS.UP)} variant='default' disabled={!isPowered} className='controller-button'>
                        <ThemeIcon variant='default' size="xl" className="controller-icon">
                            <ArrowUp />
                        </ThemeIcon>
                    </Button>
                </Flex>
                <div/>

                <Flex justify="center">
                    <Button onClick={createOnKeyClicked(DIRECTIONS.LEFT)} variant='default' disabled={!isPowered} className='controller-button'>
                        <ThemeIcon variant='default' size="xl" className="controller-icon" >
                            <ArrowLeft />
                        </ThemeIcon>
                    </Button>
                </Flex>

                <Flex justify="center">
                    <Button onClick={onPower} variant='default' className='controller-button'>
                        <ThemeIcon color={isPowered ? 'green' : 'dark'} variant='outline' size="xl" className="controller-icon">
                            <PowerButton />
                        </ThemeIcon>
                    </Button>
                </Flex>

                <Flex justify="center">
                    <Button onClick={createOnKeyClicked(DIRECTIONS.RIGHT)} variant='default' disabled={!isPowered} className='controller-button'>
                        <ThemeIcon variant='default' size="xl" className="controller-icon">
                            <ArrowRight />
                        </ThemeIcon>
                    </Button>
                </Flex>

                <div />
                <Flex justify="center">
                    <Button onClick={createOnKeyClicked(DIRECTIONS.DOWN)} variant='default' disabled={!isPowered} className='controller-button'>
                        <ThemeIcon variant='default' size="xl" className="controller-icon">
                            <ArrowDown />
                        </ThemeIcon>
                    </Button>
                </Flex>
                
            </SimpleGrid>
        </Flex>
    );
};

export default Controller;
